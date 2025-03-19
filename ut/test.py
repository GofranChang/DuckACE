import serial, threading, time, logging, json, struct, queue, traceback


class DuckAce:
    def __init__(self):
        self.serial_name = "/dev/serial/by-id/usb-ANYCUBIC_ACE_1-if00"
        self.baud = 115200
        self.__request_id = 0

        self.__read_timeout = 5

        self.__ace_pro_to_toolhead_bowden_length = 100
        self.__hub_to_toolhead_bowden_length = 100

        self.gcode = self.printer.lookup_object('gcode')
        self.saved_vars = self.printer.lookup_object('save_variables').all_variables()

        self.__filaments_info = [
            {"position": -1, "status": "empty"},
            {"position": -1, "status": "empty"},
            {"position": -1, "status": "empty"},
            {"position": -1, "status": "empty"},
        ]

        for i in range(0, 4):
            self.__filaments_info[i]["position"] = self.saved_vars.get(f"ace_filament_{i}_position", 0)

    def start(self):
        print(f"ACE: Connecting to {self.serial_name}")

        # We can catch timing where ACE reboots itself when no data is available from host. We're avoiding it with this hack
        self._connected = False
        for i in range(0, 10):
            try:
                self._serial = serial.Serial(port=self.serial_name, baudrate=self.baud)

                if self._serial.isOpen():
                    self._connected = True
                    break
            except serial.serialutil.SerialException:
                time.sleep(0.5)
                continue

        if not self._connected:
            print(f"ACE: Failed to connect to {self.serial_name}")

        print("ACE: Connected to " + self.serial_name)

        self._ace_info = None
        self._update_ace_status()


    def __update_id(self):
        if self.__request_id > 30000:
            self.__request_id = 0
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
        if not "id" in request:
            request["id"] = self.__request_id
            self.__update_id()

        payload = json.dumps(request)

        print(f"[ACE] >>> {payload}")
        payload = bytes(payload, "utf-8")

        data = bytes([0xFF, 0xAA])
        data += struct.pack("@H", len(payload))
        data += payload
        data += struct.pack("@H", self.__calc_crc(payload))
        data += bytes([0xFE])

        self._serial.write(data)
        return send_id

    def __parser_data(self, request_id, data):
        # 检查最小长度：头部(2) + 长度(2) + CRC(2) + 尾部(1)
        if len(data) < 7:
            print("数据包太短")

        # 检查头部
        if data[0:2] != b"\xff\xaa":
            print("无效的头部")

        # 解析长度字段
        payload_length = struct.unpack("@H", data[2:4])[0]

        # 提取 payload
        payload = data[4 : 4 + payload_length]

        # 提取 CRC
        crc_received = data[4 + payload_length : 4 + payload_length + 2]

        # TODO: add crc check
        # computed_crc = struct.pack('@H', self.__calc_crc(payload))
        # if computed_crc != crc_received:
        #     print("CRC 校验失败")

        try:
            json_str = payload.decode("utf-8")
            result = json.loads(json_str)
        except Exception as e:
            print("解析 JSON 出错: " + str(e))

        if result["id"] != request_id:
            illegal_id = result["id"]
            print(f"UNKNOWN ace pro id {illegal_id}, except {request_id}")

        print(f"[ACE] <<< {result}")
        return result

    def __read_serial(self):
        max_retries = 3
        retry_delay = 1
        self._data = None
        for attempt in range(max_retries):
            self._data = self._serial.read_until(expected=bytes([0xFE]), size=4096)
            if not self._data or not (
                self._data[0] == 0xFF and self._data[1] == 0xAA and self._data[len(self._data) - 1] == 0xFE
            ):
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)  # 等待一段时间后重试
            else:
                break

        return self._data

    def _get_from_ace(self):
        thread = threading.Thread(target=self.__read_serial, daemon=True)
        thread.start()
        thread.join(self.__read_timeout)

        if thread.is_alive():  # 判断是否超时
            raise TimeoutError("读取超时！")

        return self._data

    def _get_status(self):
        send_id = self.__send_to_ace({"method": "get_status"})

        data = self._get_from_ace()
        data = self.__parser_data(send_id, data)

        return data

    def _update_ace_status(self):
        self._ace_info = self._get_status()["result"]
        print(self._ace_info)

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

        data = self._get_from_ace()
        data = self.__parser_data(send_id, data)
        if data["code"] != 0 or data["msg"] != "success":
            print(
                'ACE: return error, code {}, msg "{}"'.format(data["code"], data["msg"])
            )

        self.__wait_action_finish(timeout=2)

    def __disable_feed_assist(self, index):
        send_id = self.__send_to_ace(
            {"method": "stop_feed_assist", "params": {"index": index}}
        )

        data = self._get_from_ace()
        data = self.__parser_data(send_id, data)
        if data["code"] != 0 or data["msg"] != "success":
            print(
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

        data = self._get_from_ace()
        data = self.__parser_data(send_id, data)

        if data["code"] != 0 or data["msg"] != "success":
            print(
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

        data = self._get_from_ace()
        data = self.__parser_data(send_id, data)

        if data["code"] != 0 or data["msg"] != "success":
            print(
                'ACE: return error, code {}, msg "{}"'.format(data["code"], data["msg"])
            )

        self.__wait_action_finish(timeout=(length / speed + 10))

    # TODO: Zhang Gaofan
    def cmd_ACE_GET_STATUS(self, index):
        self._get_status()

    # TODO: Zhang Gaofan
    def cmd_ACE_ENABLE_FEED_ASSIST(self, index):
        if index < 0 or index >= 4:
            print("Wrong index")

        if not self.__is_ready(index=index):
            print(
                "[ACE] Not ready, ace status is {}, channel status is {}".format(
                    self._ace_info["status"], self._ace_info["slots"][index]["status"]
                )
            )

        self.__enable_feed_assist(index)

    # TODO: Zhang Gaofan
    def cmd_ACE_DISABLE_FEED_ASSIST(self, index):
        if index < 0 or index >= 4:
            print("Wrong index")

        if not self.__is_ready(index=index):
            print(
                "[ACE] Not ready, ace status is {}, channel status is {}".format(
                    self._ace_info["status"], self._ace_info["slots"][index]["status"]
                )
            )

        self.__disable_feed_assist(index)

    def cmd_ACE_FEED(self, index, length):
        if index < 0 or index >= 4:
            print("Wrong index")

        if self.__filaments_info[index]["position"] >= self.__ace_pro_to_toolhead_bowden_length:
            error_handle()

        if not self.__is_ready(index=index):
            print(
                "[ACE] Not ready, ace status is {}, channel status is {}".format(
                    self._ace_info["status"], self._ace_info["slots"][index]["status"]
                )
            )

        # self.__enable_feed_assist(index)
        self.__feed(index, length, 50)
        # self.__disable_feed_assist(index)

        self.__filaments_info[index]["position"] += length

    def cmd_ACE_RETRACT(self, index, length):
        if index < 0 or index >= 4:
            print("Wrong index")

        if self.__filaments_info[index]["position"] <= 0:
            error_handle()

        if not self.__is_ready(index=index):
            print(
                "[ACE] Not ready, ace status is {}, channel status is {}".format(
                    self._ace_info["status"], self._ace_info["slots"][index]["status"]
                )
            )

        # self.__enable_feed_assist(index)
        self.__retract(index, length, 50)
        # self.__disable_feed_assist(index)

        self.__filaments_info[index]["position"] -= length

    def cmd_ACE_CALIBRATE_BOWDEN_LENGTH(self, index):
        pass

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

    def cmd_GET_FILAMENT_INFO(self):
        print(f"[ACE] Filament info : {self.__filaments_info}")


if __name__ == "__main__":
    ace = DuckAce()
    ace.start()

    ace.cmd_GET_FILAMENT_INFO()

    # ace.cmd_ACE_FEED(0, 20)
    time.sleep(1)
    ace.cmd_ACE_RETRACT(2, 200)
    time.sleep(1)
    # ace.cmd_ACE_FEED(2, 20)

    ace.cmd_GET_FILAMENT_INFO()
