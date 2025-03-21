import serial, time, json, struct
import logging


class DuckAce:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')

        self.serial_name = config.get("serial", "/dev/serial/by-id/usb-ANYCUBIC_ACE_1-if00")
        self.baud = config.getint('baud', 115200)
        self.__request_id = 0

        extruder_sensor_pin = config.get('extruder_sensor_pin', None)
        toolhead_sensor_pin = config.get('toolhead_sensor_pin', None)
        self.__create_mmu_sensor(config, extruder_sensor_pin, "extruder_sensor")
        self.__create_mmu_sensor(config, toolhead_sensor_pin, "toolhead_sensor")

        self.__feed_speed = int(config.get('feed_speed', 50))
        self.__retract_speed = int(config.get('retract_speed', 50))

        # self.__read_timeout = 5

        self.__ace_pro_to_toolhead_bowden_length = 100
        self.__hub_to_toolhead_bowden_length = 100

        # self.gcode = self.printer.lookup_object('gcode')
        # self.saved_vars = self.printer.lookup_object('save_variables').all_variables()

        self.__filaments_info = [
            {"position": -1, "status": "empty"},
            {"position": -1, "status": "empty"},
            {"position": -1, "status": "empty"},
            {"position": -1, "status": "empty"},
        ]

        # for i in range(0, 4):
        #     self.__filaments_info[i]["position"] = self.saved_vars.get(f"ace_filament_{i}_position", 0)

        self.printer.register_event_handler('klippy:ready', self.start)
        self.printer.register_event_handler('klippy:disconnect', self.stop)

        # self.gcode.register_command(
        #     "cmd_ACE_GET_FILAMENT_INFO", self.cmd_ACE_GET_FILAMENT_INFO,
        #     desc=self.cmd_ACE_GET_FILAMENT_INFO_help)

        self.gcode.register_command(
            'ACE_FEED', self.cmd_ACE_FEED,
            desc=self.cmd_ACE_FEED_help)
        self.gcode.register_command(
            'ACE_RETRACT', self.cmd_ACE_RETRACT,
            desc=self.cmd_ACE_RETRACT_help)
        self.gcode.register_command(
            'ACE_CALIBRATE_BOWDEN_LENGTH', self.cmd_ACE_CALIBRATE_BOWDEN_LENGTH,
            desc=self.cmd_ACE_CALIBRATE_BOWDEN_LENGTH_help)
        self.gcode.register_command(
            'ACE_GET_INFO', self.cmd_ACE_GET_INFO,
            desc=self.cmd_ACE_GET_INFO_help)

    def start(self):
        logging.info(f"ACE: Connecting to {self.serial_name}")

        # We can catch timing where ACE reboots itself when no data is available from host. We're avoiding it with this hack
        self._connected = False
        for i in range(0, 10):
            try:
                self._serial = serial.Serial(
                    port=self.serial_name, 
                    baudrate=self.baud, 
                    timeout=2,         # 读超时 2 秒
                    write_timeout=2
                )

                if self._serial.isOpen():
                    self._connected = True
                    break
            except serial.serialutil.SerialException:
                time.sleep(0.5)
                continue

        if not self._connected:
            logging.error(f"ACE: Failed to connect to {self.serial_name}")

        logging.info("ACE: Connected to " + self.serial_name)

        self._ace_info = None
        self._update_ace_status()

    def stop(self):
        pass

    def __reconnect_serial(self, max_attempts=3, delay=1):
        # 重连函数，最多重试 max_attempts 次
        for attempt in range(max_attempts):
            try:
                logging.info(f"Attempt {attempt+1} to reconnect...")
                self._serial = serial.Serial(port=self.serial_name,
                                             baudrate=self.baud,
                                             timeout=2,
                                             write_timeout=2)
                if self._serial.isOpen():
                    logging.info("Reconnected successfully.")
                    return
            except Exception as e:
                logging.info(f"Reconnect attempt {attempt+1} failed: {e}")
            time.sleep(delay)
        raise Exception("Failed to reconnect to serial port.")

    def __create_mmu_sensor(self, config, pin, name):
        section = "filament_switch_sensor %s" % name
        config.fileconfig.add_section(section)
        config.fileconfig.set(section, "switch_pin", pin)
        config.fileconfig.set(section, "pause_on_runout", "False")
        fs = self.printer.load_object(config, section)

    def __update_id(self):
        if self.__request_id >= 16382:
            self.__reconnect_serial()
            self.__request_id = 16380
        else:
            self.__request_id += 1

    def __calc_crc(self, buffer):
        _crc = 0xFFFF
        for byte in buffer:
            data = byte
            data ^= _crc & 0xFF
            data ^= (data & 0x0F) << 4
            _crc = ((data << 8) | (_crc >> 8)) ^ (data >> 4) ^ (data << 3)
        return _crc

    def __send_to_ace(self, request):
        send_id = self.__request_id
        if "id" not in request:
            request["id"] = self.__request_id
            self.__update_id()

        payload = json.dumps(request)

        logging.debug(f"[ACE] >>> {payload}")
        payload = bytes(payload, "utf-8")

        data = bytes([0xFF, 0xAA])
        data += struct.pack("@H", len(payload))
        data += payload
        data += struct.pack("@H", self.__calc_crc(payload))
        data += bytes([0xFE])

        try:
            self._serial.write(data)
        except Exception as e:
            logging.info(f"Serial write failed: {e}, attempting to reconnect...")
            try:
                self.__reconnect_serial()
                self._serial.write(data)
                logging.info("Resend successful after reconnection.")
            except Exception as e2:
                logging.info(f"Resend failed after reconnection: {e2}")
                raise e2

        return send_id

    def __parser_data(self, request_id, data):
        # 检查最小长度：头部(2) + 长度(2) + CRC(2) + 尾部(1)
        if len(data) < 7:
            logging.info("数据包太短")
            return None

        # 检查头部
        if data[0:2] != b"\xff\xaa":
            logging.info("无效的头部")
            return None

        # 解析长度字段
        payload_length = struct.unpack("@H", data[2:4])[0]

        # 提取 payload
        payload = data[4 : 4 + payload_length]

        # 提取 CRC
        crc_received = data[4 + payload_length : 4 + payload_length + 2]

        # TODO: add crc check
        # computed_crc = struct.pack('@H', self.__calc_crc(payload))
        # if computed_crc != crc_received:
        #     logging.info("CRC 校验失败")

        try:
            json_str = payload.decode("utf-8")
            result = json.loads(json_str)
        except Exception as e:
            logging.info("解析 JSON 出错: " + str(e))

        if result["id"] != request_id:
            illegal_id = result["id"]
            logging.info(f"UNKNOWN ace pro id {illegal_id}, except {request_id}")

        logging.info(f"[ACE] <<< {result}")
        return result

    def __get_from_ace(self):
        max_retries = 3
        retry_delay = 1
        self._data = None
        for attempt in range(max_retries):
            try:
                self._data = self._serial.read_until(expected=bytes([0xFE]), size=4096)
            except Exception as e:
                try:
                    self.__reconnect_serial()
                    self._data = self._serial.read_until(expected=bytes([0xFE]), size=4096)
                    logging.info("Resend successful after reconnection.")
                except Exception as e2:
                    logging.info(f"Resend failed after reconnection: {e2}")
                    raise e2
            if not self._data or not (
                self._data[0] == 0xFF and self._data[1] == 0xAA and self._data[len(self._data) - 1] == 0xFE
            ):
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)  # 等待一段时间后重试
            else:
                break

        return self._data

    def _get_status(self):
        send_id = self.__send_to_ace({"method": "get_status"})

        data = self.__get_from_ace()
        data = self.__parser_data(send_id, data)

        return data

    def _update_ace_status(self):
        self._ace_info = self._get_status()["result"]
        logging.info(self._ace_info)

        for i in range(0, 4):
            if self.__filaments_info[i]["status"] == "empty" and self._ace_info["slots"][i]["status"] == "ready":
                self.__filaments_info[i]["position"] = 0

            self.__filaments_info[i]["status"] = self._ace_info["slots"][i]["status"]

    def __is_ready(self, index=None):
        self._update_ace_status()

        if index == None:
            return (
                self._ace_info["slots"][index]["status"] == "ready"
                and self._ace_info["slots"][index]["status"] == "ready"
            )
        else:
            return self._ace_info["slots"][index]["status"] == "ready"
        
    def __loop_get_status(self):
        while True:
            data = self._get_status()["result"]
            if data["status"] != "busy":
                break
            time.sleep(0.5)

    def __wait_action_finish(self, timeout):
        start_time = time.localtime().tm_sec
        
        while True:
            if time.localtime().tm_sec - start_time > timeout:
                # TODO: Timeout handle
                errorhandle()

            data = self._get_status()["result"]
            if data["status"] != "busy":
                break

            time.sleep(0.5)

    def __enable_feed_assist(self, index):
        send_id = self.__send_to_ace(
            {"method": "start_feed_assist", "params": {"index": index}}
        )

        data = self.__get_from_ace()
        data = self.__parser_data(send_id, data)
        if data["code"] != 0 or data["msg"] != "success":
            logging.info(
                'ACE: return error, code {}, msg "{}"'.format(data["code"], data["msg"])
            )

        self.__wait_action_finish(timeout=2)

    def __disable_feed_assist(self, index):
        send_id = self.__send_to_ace(
            {"method": "stop_feed_assist", "params": {"index": index}}
        )

        data = self.__get_from_ace()
        data = self.__parser_data(send_id, data)
        if data["code"] != 0 or data["msg"] != "success":
            logging.info(
                'ACE: return error, code {}, msg "{}"'.format(data["code"], data["msg"])
            )

        self.__wait_action_finish(timeout=2)

    def __feed(self, index, length, speed):
        send_id = self.__send_to_ace(
            {
                "method": "feed_filament",
                "params": {"index": index, "length": length, "speed": speed},
            }
        )

        data = self.__get_from_ace()
        data = self.__parser_data(send_id, data)

        if data["code"] != 0 or data["msg"] != "success":
            logging.info(
                'ACE: return error, code {}, msg "{}"'.format(data["code"], data["msg"])
            )

        self.__wait_action_finish(timeout=(length / speed + 10))

    def __retract(self, index, length, speed):
        send_id = self.__send_to_ace(
            {
                "method": "unwind_filament",
                "params": {"index": index, "length": length, "speed": speed},
            }
        )

        data = self.__get_from_ace()
        data = self.__parser_data(send_id, data)

        if data["code"] != 0 or data["msg"] != "success":
            logging.info(
                'ACE: return error, code {}, msg "{}"'.format(data["code"], data["msg"])
            )

        self.__wait_action_finish(timeout=(length / speed + 10))

    # TODO: Zhang Gaofan
    def cmd_ACE_GET_STATUS(self, index):
        self._get_status()

    # TODO: Zhang Gaofan
    def cmd_ACE_ENABLE_FEED_ASSIST(self, index):
        if index < 0 or index >= 4:
            logging.info("Wrong index")

        if not self.__is_ready(index=index):
            logging.info(
                "[ACE] Not ready, ace status is {}, channel status is {}".format(
                    self._ace_info["status"], self._ace_info["slots"][index]["status"]
                )
            )

        self.__enable_feed_assist(index)

    # TODO: Zhang Gaofan
    def cmd_ACE_DISABLE_FEED_ASSIST(self, index):
        if index < 0 or index >= 4:
            logging.info("Wrong index")

        if not self.__is_ready(index=index):
            logging.info(
                "[ACE] Not ready, ace status is {}, channel status is {}".format(
                    self._ace_info["status"], self._ace_info["slots"][index]["status"]
                )
            )

        self.__disable_feed_assist(index)

    cmd_ACE_FEED_help = 'ACE Feed'
    def cmd_ACE_FEED(self, gcmd):
        index = gcmd.get_int('CHANNEL')
        length = gcmd.get_float('LENGTH')
        force = gcmd.get_int('FORCE')

        if index < 0 or index >= 4:
            logging.info("Wrong index")

        if not force and self.__filaments_info[index]["position"] >= self.__ace_pro_to_toolhead_bowden_length:
            error_handle()

        if not self.__is_ready(index=index):
            logging.info(
                "[ACE] Not ready, ace status is {}, channel status is {}".format(
                    self._ace_info["status"], self._ace_info["slots"][index]["status"]
                )
            )

        self.__feed(index, length, self.__feed_speed)
        self.__filaments_info[index]["position"] += length

    cmd_ACE_RETRACT_help = 'ACE Retract'
    def cmd_ACE_RETRACT(self, gcmd):
        index = gcmd.get_int('CHANNEL')
        length = gcmd.get_float('LENGTH')
        force = gcmd.get_int('FORCE')

        if index < 0 or index >= 4:
            logging.info("Wrong index")

        if not force and self.__filaments_info[index]["position"] <= 0:
            error_handle()

        if not self.__is_ready(index=index):
            logging.info(
                "[ACE] Not ready, ace status is {}, channel status is {}".format(
                    self._ace_info["status"], self._ace_info["slots"][index]["status"]
                )
            )

        self.__retract(index, length, self.__retract_speed)
        self.__filaments_info[index]["position"] -= length

    cmd_ACE_CALIBRATE_BOWDEN_LENGTH_help = 'ACE Debug'
    def cmd_ACE_CALIBRATE_BOWDEN_LENGTH(self, gcmd):
        index = gcmd.get_int('CHANNEL')
        length = gcmd.get_int('LENGTH')

        sensor_extruder = self.printer.lookup_object("filament_switch_sensor %s" % "extruder_sensor", None)

        self.__feed(index, length, self.__feed_speed)
        bowden_length = length
        while not bool(sensor_extruder.runout_helper.filament_present):
            self.__feed(index, 10, 10)
            bowden_length += 2
            time.sleep(0.5)
            self.gcode.respond_info(f"[ACE] channel {index} bowden length {bowden_length} {sensor_extruder.runout_helper}")

    def cmd_ACE_CALIBRATE_HUB_TO_TOOLHEAD_BOWON_LENGTH(self):
        pass

    def cmd_CHANGE_FILAMENT(self, to_index):
        from_index = -1

        for i in range(0, 4):
            if self.__filaments_info[i]["position"] - self.__ace_pro_to_toolhead_bowden_length <= 0:
                self.cmd_ACE_RETRACT(i, self.__filaments_info[i]["position"])
                from_index = i

        if from_index == to_index:
            return

        if from_index != -1:
            self.cmd_ACE_RETRACT(from_index, self.__hub_to_toolhead_bowden_length + 20)

        self.cmd_ACE_FEED(to_index, self.__hub_to_toolhead_bowden_length - 10)

    cmd_ACE_GET_INFO_help = 'ACE Debug'
    def cmd_ACE_GET_INFO(self, gcmd):
        logging.info(f"[ACE] Filament info : {self.__filaments_info}")
        self.gcode.respond_info(f"[ACE] filament info {self.__filaments_info}")

def load_config(config):
    return DuckAce(config)
