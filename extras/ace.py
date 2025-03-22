import serial, time, logging, json, struct, threading

class DuckAce:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self._name = config.get_name()
        if self._name.startswith('ace '):
            self._name = self._name[4:]
        self.variables = self.printer.lookup_object('save_variables').allVariables


        self.serial_name = config.get('serial', '/dev/ttyACM0')
        self.baud = config.getint('baud', 115200)
        extruder_sensor_pin = config.get('extruder_sensor_pin', None)
        toolhead_sensor_pin = config.get('toolhead_sensor_pin', None)
        self.feed_speed = config.getint('feed_speed', 50)
        self.retract_speed = config.getint('retract_speed', 50)
        self.toolchange_retract_length = config.getint('toolchange_retract_length', 100)
        self.max_dryer_temperature = config.getint('max_dryer_temperature', 55)

        self._callback_map = {}
        self.park_hit_count = 5
        self._feed_assist_index = -1
        self._request_id = 0
        self._last_assist_count = 0
        self._assist_hit_count = 0
        self._park_in_progress = False
        self._park_is_toolchange = False
        self._park_previous_tool = -1
        self._park_index = -1

        # Default data to prevent exceptions
        self._info = {
            'status': 'ready',
            'dryer': {
                'status': 'stop',
                'target_temp': 0,
                'duration': 0,
                'remain_time': 0
            },
            'temp': 0,
            'enable_rfid': 1,
            'fan_speed': 7000,
            'feed_assist_count': 0,
            'cont_assist_time': 0.0,
            'slots': [
                {
                    'index': 0,
                    'status': 'empty',
                    'sku': '',
                    'type': '',
                    'color': [0, 0, 0]
                },
                {
                    'index': 1,
                    'status': 'empty',
                    'sku': '',
                    'type': '',
                    'color': [0, 0, 0]
                },
                {
                    'index': 2,
                    'status': 'empty',
                    'sku': '',
                    'type': '',
                    'color': [0, 0, 0]
                },
                {
                    'index': 3,
                    'status': 'empty',
                    'sku': '',
                    'type': '',
                    'color': [0, 0, 0]
                }
            ]
        }

        self._create_mmu_sensor(config, extruder_sensor_pin, "extruder_sensor")
        self._create_mmu_sensor(config, toolhead_sensor_pin, "toolhead_sensor")
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler('klippy:disconnect', self._handle_disconnect)

        self.gcode.register_command(
            'ACE_START_DRYING', self.cmd_ACE_START_DRYING,
            desc=self.cmd_ACE_START_DRYING_help)
        self.gcode.register_command(
            'ACE_STOP_DRYING', self.cmd_ACE_STOP_DRYING,
            desc=self.cmd_ACE_STOP_DRYING_help)
        self.gcode.register_command(
            'ACE_ENABLE_FEED_ASSIST', self.cmd_ACE_ENABLE_FEED_ASSIST,
            desc=self.cmd_ACE_ENABLE_FEED_ASSIST_help)
        self.gcode.register_command(
            'ACE_DISABLE_FEED_ASSIST', self.cmd_ACE_DISABLE_FEED_ASSIST,
            desc=self.cmd_ACE_DISABLE_FEED_ASSIST_help)
        self.gcode.register_command(
            'ACE_FEED', self.cmd_ACE_FEED,
            desc=self.cmd_ACE_FEED_help)
        self.gcode.register_command(
            'ACE_RETRACT', self.cmd_ACE_RETRACT,
            desc=self.cmd_ACE_RETRACT_help)
        self.gcode.register_command(
            'ACE_CHANGE_TOOL', self.cmd_ACE_CHANGE_TOOL,
            desc=self.cmd_ACE_CHANGE_TOOL_help)
        self.gcode.register_command(
            'ACE_FILAMENT_STATUS', self.cmd_ACE_FILAMENT_STATUS,
            desc=self.cmd_ACE_FILAMENT_STATUS_help)
        self.gcode.register_command(
            'ACE_GET_INFO', self.cmd_ACE_GET_INFO,
            desc=self.cmd_ACE_GET_INFO_help)
        self.gcode.register_command(
            'ACE_DEBUG', self.cmd_ACE_DEBUG,
            desc=self.cmd_ACE_DEBUG_help)



    def _calc_crc(self, buffer):
        _crc = 0xffff
        for byte in buffer:
            data = byte
            data ^= _crc & 0xff
            data ^= (data & 0x0f) << 4
            _crc = ((data << 8) | (_crc >> 8)) ^ (data >> 4) ^ (data << 3)
        return _crc



    def _reconnect_serial(self, max_attempts=3, delay=1):
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




    def _handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')

        logging.info('ACE: Connecting to ' + self.serial_name)

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
            logging.error(f'ACE: Failed to connect to {self.serial_name}')

        logging.info(f'ACE: Connected to {self.serial_name}')

        self._update_ace_status()


    def _handle_disconnect(self):
        logging.info('ACE: Closing connection to ' + self.serial_name)
        self._serial.close()
        self._connected = False
        self._reader_thread.join()
        self._writer_thread.join()
        self.reactor.unregister_timer(self.main_timer)

        self._queue = None
        self._main_queue = None

    def _update_id(self):
        if self._request_id >= 16382:
            self._reconnect_serial()
            self._request_id = 16380
        else:
            self._request_id += 1

    def _send_to_ace(self, request):
        send_id = self._request_id
        if "id" not in request:
            request["id"] = self._request_id
            self._update_id()

        payload = json.dumps(request)

        logging.debug(f"[ACE] >>> {payload}")
        payload = bytes(payload, "utf-8")

        data = bytes([0xFF, 0xAA])
        data += struct.pack("@H", len(payload))
        data += payload
        data += struct.pack("@H", self._calc_crc(payload))
        data += bytes([0xFE])

        # self.gcode.respond_info(f"[ACE] >>> {request}")
        try:
            self._serial.write(data)
        except Exception as e:
            logging.info(f"Serial write failed: {e}, attempting to reconnect...")
            try:
                self._reconnect_serial()
                self._serial.write(data)
                logging.info("Resend successful after reconnection.")
            except Exception as e2:
                logging.info(f"Resend failed after reconnection: {e2}")
                raise e2

        return send_id

    def _get_from_ace(self):
        max_retries = 3
        retry_delay = 1
        self._data = None
        for attempt in range(max_retries):
            try:
                self._data = self._serial.read_until(expected=bytes([0xFE]), size=4096)
            except Exception as e:
                try:
                    self._reconnect_serial()
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

        # self.gcode.respond_info(f"[ACE] <<< {self._data}")
        return self._data
    
    def _parser_data(self, data):
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
        # computed_crc = struct.pack('@H', self._calc_crc(payload))
        # if computed_crc != crc_received:
        #     logging.info("CRC 校验失败")

        try:
            json_str = payload.decode("utf-8")
            result = json.loads(json_str)
        except Exception as e:
            logging.info("解析 JSON 出错: " + str(e))

        logging.info(f"[ACE] <<< {result}")
        return result
    
    def _get_ace_ack(self, send_id):
        data = self._get_from_ace()
        data = self._parser_data(data)

        if None == data["id"]:
            return None

        if data["id"] != send_id:
            self.gcode.respond_info('Get UNKNOWN response id ' + data["id"])
            return None
        
        return data

    def _get_status(self):
        send_id = self._send_to_ace({"method": "get_status"})

        data = self._get_ace_ack(send_id)
        return data

    def _update_ace_status(self):
        if None == self._get_status()["result"]:
            return False
        self._info = self._get_status()["result"]
        self.gcode.respond_info(f'[ACE] status {self._info}')
        return True


    def dwell(self, delay = 1.):
        self.toolhead.dwell(delay)


    def _extruder_move(self, length, speed):
        pos = self.toolhead.get_position()
        pos[3] += length
        self.toolhead.move(pos, speed)
        return pos[3]

    def _create_mmu_sensor(self, config, pin, name):
        section = "filament_switch_sensor %s" % name
        config.fileconfig.add_section(section)
        config.fileconfig.set(section, "switch_pin", pin)
        config.fileconfig.set(section, "pause_on_runout", "False")
        fs = self.printer.load_object(config, section)

    def _wait_ace_ready(self, timeout=10):
        self.gcode.respond_info(f"Wait ace ready, timeout {timeout}...")
        start_time = time.time()

        while time.time() - start_time < timeout:
            if not self._update_ace_status():
                self._update_ace_status()

            if self._info['status'] == 'ready':
                self.gcode.respond_info("ACE ready")
                return True
            
            remaining = timeout - (time.time() - start_time)
            self.gcode.respond_info(f"Wait ace ready: time left {remaining:.1f}s")
            time.sleep(1)  # 使用标准sleep释放GIL
        
        self.gcode.respond_error("Wait ace ready timeout")
        return False


    cmd_ACE_START_DRYING_help = 'Starts ACE Pro dryer'
    def cmd_ACE_START_DRYING(self, gcmd):
        temperature = gcmd.get_int('TEMP')
        duration = gcmd.get_int('DURATION', 240)

        if duration <= 0:
            raise gcmd.error('Wrong duration')
        if temperature <= 0 or temperature > self.max_dryer_temperature:
            raise gcmd.error('Wrong temperature')

        send_id = self._send_to_ace(request={"method": "drying", "params": {"temp":temperature, "fan_speed": 7000, "duration": duration}})
        self._get_ace_ack(send_id)

        wait_thread = threading.Thread(
            target=self._wait_ace_ready,
            args=(10,),
            daemon=True
        )
        wait_thread.start()
        wait_thread.join()

        self.gcode.respond_info(f'ACE Start drying success')


    cmd_ACE_STOP_DRYING_help = 'Stops ACE Pro dryer'
    def cmd_ACE_STOP_DRYING(self, gcmd):
        send_id = self._send_to_ace(request = {"method":"drying_stop"})
        self._get_ace_ack(send_id)

        wait_thread = threading.Thread(
            target=self._wait_ace_ready,
            args=(10,),
            daemon=True
        )
        wait_thread.start()
        wait_thread.join()

        self.gcode.respond_info("ACE Stop drying finish")

    def _start_feed_assist(self, index):
        send_id = self._send_to_ace(request={"method": "start_feed_assist", "params": {"index": index}})
        self._get_ace_ack(send_id)

        wait_thread = threading.Thread(
            target=self._wait_ace_ready,
            args=(10,),
            daemon=True
        )
        wait_thread.start()
        wait_thread.join()

        self.gcode.respond_info(f"ACE start channel {index} feed assist success")

    cmd_ACE_ENABLE_FEED_ASSIST_help = 'Enables ACE feed assist'
    def cmd_ACE_ENABLE_FEED_ASSIST(self, gcmd):
        index = gcmd.get_int('INDEX')

        if index < 0 or index >= 4:
            raise gcmd.error('Wrong index')

        self._start_feed_assist(index)


    def _stop_feed_assist(self, index):
        send_id = self._send_to_ace(request={"method": "stop_feed_assist", "params": {"index": index}})
        self._get_ace_ack(send_id)

        wait_thread = threading.Thread(
            target=self._wait_ace_ready,
            args=(10,),
            daemon=True
        )
        wait_thread.start()
        wait_thread.join()

        self.gcode.respond_info(f"ACE stop channel {index} feed assist success")

    cmd_ACE_DISABLE_FEED_ASSIST_help = 'Disables ACE feed assist'
    def cmd_ACE_DISABLE_FEED_ASSIST(self, gcmd):
        if self._feed_assist_index != -1:
            index = gcmd.get_int('INDEX', self._feed_assist_index)
        else:
            index = gcmd.get_int('INDEX')

        if index < 0 or index >= 4:
            raise gcmd.error('Wrong index')

        self._stop_feed_assist(index)



    def _feed(self, index, length, speed):
        send_id = self._send_to_ace(request={"method": "feed_filament", "params": {"index": index, "length": length, "speed": speed}})
        self._get_ace_ack(send_id)
        
        wait_thread = threading.Thread(
            target=self._wait_ace_ready,
            args=(length / speed + 10,),
            daemon=True
        )
        wait_thread.start()
        wait_thread.join()

        self.gcode.respond_info(f"ACE feed channel {index} length {length} speed {speed}")

    cmd_ACE_FEED_help = 'Feeds filament from ACE'
    def cmd_ACE_FEED(self, gcmd):
        index = gcmd.get_int('INDEX')
        length = gcmd.get_int('LENGTH')
        speed = gcmd.get_int('SPEED', self.feed_speed)

        if index < 0 or index >= 4:
            raise gcmd.error('Wrong index')
        if length <= 0:
            raise gcmd.error('Wrong length')
        if speed <= 0:
            raise gcmd.error('Wrong speed')

        self._feed(index, length, speed)


    def _retract(self, index, length, speed):
        send_id = self._send_to_ace(request={"method": "unwind_filament", "params": {"index": index, "length": length, "speed": speed}})
        self._get_ace_ack(send_id)
        
        wait_thread = threading.Thread(
            target=self._wait_ace_ready,
            args=(length / speed + 10,),
            daemon=True
        )
        wait_thread.start()
        wait_thread.join()

        self.gcode.respond_info(f"ACE unwind_filament channel {index} length {length} speed {speed}")


    cmd_ACE_RETRACT_help = 'Retracts filament back to ACE'
    def cmd_ACE_RETRACT(self, gcmd):
        index = gcmd.get_int('INDEX')
        length = gcmd.get_int('LENGTH')
        speed = gcmd.get_int('SPEED', self.retract_speed)

        if index < 0 or index >= 4:
            raise gcmd.error('Wrong index')
        if length <= 0:
            raise gcmd.error('Wrong length')
        if speed <= 0:
            raise gcmd.error('Wrong speed')

        self._retract(index, length, speed)

    def _park_to_toolhead(self, tool):

        sensor_extruder = self.printer.lookup_object("filament_switch_sensor %s" % "extruder_sensor", None)
        sensor_toolhead = self.printer.lookup_object("filament_switch_sensor %s" % "toolhead_sensor", None)
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()

        self._start_feed_assist(tool)

        while not bool(sensor_extruder.runout_helper.filament_present):
            self.dwell(delay=0.1)

        if not bool(sensor_extruder.runout_helper.filament_present):
            raise ValueError("Filament stuck " + str(bool(sensor_extruder.runout_helper.filament_present)))
        else:
            self.variables['ace_filament_pos'] = "spliter"

        while not bool(sensor_toolhead.runout_helper.filament_present):
            self._extruder_move(1, 5)

        self.variables['ace_filament_pos'] = "toolhead"

        self._extruder_move(50, 5)
        self.variables['ace_filament_pos'] = "nozzle"

    cmd_ACE_CHANGE_TOOL_help = 'Changes tool'
    def cmd_ACE_CHANGE_TOOL(self, gcmd):
        tool = gcmd.get_int('TOOL')
        sensor_extruder = self.printer.lookup_object("filament_switch_sensor %s" % "extruder_sensor", None)

        if tool < -1 or tool >= 4:
            raise gcmd.error('Wrong tool')

        was = self.variables.get('ace_current_index', -1)
        if was == tool:
            gcmd.respond_info('ACE: Not changing tool, current index already ' + str(tool))
            return

        if tool != -1:
            status = self._info['slots'][tool]['status']
            if status != 'ready':
                self.gcode.run_script_from_command('_ACE_ON_EMPTY_ERROR INDEX=' + str(tool))
                return

        self.gcode.run_script_from_command('_ACE_PRE_TOOLCHANGE FROM=' + str(was) + ' TO=' + str(tool))


        self.gcode.respond_info('ACE: Toolchange ' + str(was) + ' => ' + str(tool))
        logging.info('ACE: Toolchange ' + str(was) + ' => ' + str(tool))
        if was != -1:
            self._stop_feed_assist(was)
            self.wait_ace_ready()
            if  self.variables.get('ace_filament_pos', "spliter") == "nozzle":
                self.gcode.run_script_from_command('CUT_TIP')
                self.variables['ace_filament_pos'] = "toolhead"

            if  self.variables.get('ace_filament_pos', "spliter") == "toolhead":
                while bool(sensor_extruder.runout_helper.filament_present):
                    self._extruder_move(-20, 5)
                    self._retract(was, 20, self.retract_speed)
                    self.dwell(1)
                self.variables['ace_filament_pos'] = "bowden"

            self.wait_ace_ready()

            self._retract(was, self.toolchange_retract_length, self.retract_speed)
            self.variables['ace_filament_pos'] = "spliter"

            self.wait_ace_ready()

            if tool != -1:

                self._feed(tool, self.toolchange_retract_length-5, self.retract_speed)
                self.variables['ace_filament_pos'] = "bowden"

                self.wait_ace_ready()

                self._park_to_toolhead(tool)
        else:
            self._park_to_toolhead(tool)

        self.gcode.run_script_from_command('_ACE_POST_TOOLCHANGE FROM=' + str(was) + ' TO=' + str(tool))

        self.variables['ace_current_index'] = tool
        # Force save to disk
        self.gcode.run_script_from_command('SAVE_VARIABLE VARIABLE=ace_current_index VALUE=' + str(tool))
        self.gcode.run_script_from_command(f"""SAVE_VARIABLE VARIABLE=ace_filament_pos VALUE='"{self.variables['ace_filament_pos']}"'""")

        gcmd.respond_info(f"Tool {tool} load")


    cmd_ACE_FILAMENT_STATUS_help = 'ACE Filament status'
    def cmd_ACE_FILAMENT_STATUS(self, gcmd):
        sensor_extruder = self.printer.lookup_object("filament_switch_sensor %s" % "extruder_sensor", None)
        sensor_toolhead = self.printer.lookup_object("filament_switch_sensor %s" % "toolhead_sensor", None)
        state = "ACE----------|*--|Ex--|*----|Nz--"
        if  self.variables['ace_filament_pos'] == "nozzle":
            state = "ACE>>>>>>>>>>|*>>|Ex>>|*>>|Nz>>"
        if  self.variables['ace_filament_pos'] == "toolhead" and bool(sensor_toolhead.runout_helper.filament_present):
            state = "ACE>>>>>>>>>>|*>>|Ex>>|*>>|Nz--"
        if  self.variables['ace_filament_pos'] == "toolhead" and not bool(sensor_toolhead.runout_helper.filament_present):
            state = "ACE>>>>>>>>>>|*>>|Ex>>|*--|Nz--"
        if  self.variables['ace_filament_pos'] == "bowden" and bool(sensor_extruder.runout_helper.filament_present):
            state = "ACE>>>>>>>>>>|*>>|Ex--|*--|Nz--"
        if  self.variables['ace_filament_pos'] == "bowden" and not bool(sensor_extruder.runout_helper.filament_present):
            state = "ACE>>>>>>>>>>|*--|Ex--|*--|Nz--"
        gcmd.respond_info(state)

    cmd_ACE_GET_INFO_help = 'ACE Get info'
    def cmd_ACE_GET_INFO(self, gcmd):
        gcmd.respond_info(f'[ACE] Info {self._info}')

    cmd_ACE_DEBUG_help = 'ACE Debug'
    def cmd_ACE_DEBUG(self, gcmd):
        method = gcmd.get('METHOD')
        params = gcmd.get('PARAMS', '{}')

        try:
            def callback(self, response):
                self.gcode.respond_info(str(response))

            self.send_request(request = {"method": method, "params": json.loads(params)}, callback = callback)
        except Exception as e:
            self.gcode.respond_info('Error: ' + str(e))


def load_config(config):
    return DuckAce(config)
