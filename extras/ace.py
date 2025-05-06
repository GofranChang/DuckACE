import serial, time, logging, json, struct, queue, traceback # type: ignore
from datetime import datetime

class PeekableQueue(queue.Queue):
    def peek(self):
        with self.mutex:
            if len(self.queue) == 0:
                return None
            return self.queue[0]

class DuckAce:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')

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
        self._feed_assist_channel = -1
        self._last_assist_count = 0
        self._assist_hit_count = 0
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

        self._create_mmu_sensor(config, extruder_sensor_pin, 'extruder_sensor')
        self._create_mmu_sensor(config, toolhead_sensor_pin, 'toolhead_sensor')

        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler('klippy:disconnect', self._handle_disconnect)

        self.gcode.register_command(
            'ACE_GET_CUR_INDEX', self.cmd_ACE_GET_CUR_INDEX,
            desc=self.cmd_ACE_GET_CUR_INDEX_help
        )
        self.gcode.register_command(
            'ACE_START_DRYING', self.cmd_ACE_START_DRYING,
            desc=self.cmd_ACE_START_DRYING_help
        )
        self.gcode.register_command(
            'ACE_STOP_DRYING', self.cmd_ACE_STOP_DRYING,
            desc=self.cmd_ACE_STOP_DRYING_help
        )
        self.gcode.register_command(
            'ACE_ENABLE_FEED_ASSIST', self.cmd_ACE_ENABLE_FEED_ASSIST,
            desc=self.cmd_ACE_ENABLE_FEED_ASSIST_help
        )
        self.gcode.register_command(
            'ACE_DISABLE_FEED_ASSIST', self.cmd_ACE_DISABLE_FEED_ASSIST,
            desc=self.cmd_ACE_DISABLE_FEED_ASSIST_help
        )
        self.gcode.register_command(
            'ACE_FEED', self.cmd_ACE_FEED,
            desc=self.cmd_ACE_FEED_help
        )
        self.gcode.register_command(
            'ACE_RETRACT', self.cmd_ACE_RETRACT,
            desc=self.cmd_ACE_RETRACT_help
        )
        self.gcode.register_command(
            'ACE_REJECT_TOOL', self.cmd_ACE_REJECT_TOOL,
            desc=self.cmd_ACE_REJECT_TOOL_help
        )
        self.gcode.register_command(
            'ACE_CHANGE_TOOL', self.cmd_ACE_CHANGE_TOOL,
            desc=self.cmd_ACE_CHANGE_TOOL_help
        )
        self.gcode.register_command(
            'ACE_FILAMENT_STATUS', self.cmd_ACE_FILAMENT_STATUS,
            desc=self.cmd_ACE_FILAMENT_STATUS_help
        )
        self.gcode.register_command(
            'ACE_CLEAR_ALL_STATUS', self.cmd_ACE_CLEAR_ALL_STATUS,
            desc=self.cmd_ACE_CLEAR_ALL_STATUS_help
        )
        self.gcode.register_command(
            'ACE_DEBUG', self.cmd_ACE_DEBUG,
            desc=self.cmd_ACE_DEBUG_help
        )

    def _handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')

        logging.info(f'ACE: Connecting to {self.serial_name}')

        self._request_id = 0
        self._connected = False
        self._serial = None

        start_time = time.time()
        while not self._connected:
            if time.time() - start_time > 30:
                raise TimeoutError('Connect to ACE failed')

            self._reconnect_serial()
            self.reactor.pause(0.5)

        if not self._connected:
            raise ValueError(f'ACE: Failed to connect to {self.serial_name}')

        logging.info(f'ACE: Connected to {self.serial_name}')

        self._serial_task_queue = PeekableQueue()
        self.serial_timer = self.reactor.register_timer(self._serial_read_write, self.reactor.NOW)

        def info_callback(self, response):
            res = response['result']
            self.gcode.respond_info('Connected ' + res['model'] + ' ' + res['firmware'])
        self.send_request(request = {'method': 'get_info'}, callback = info_callback)


    def _handle_disconnect(self):
        logging.info('ACE: Closing connection to ' + self.serial_name)
        self._serial.close()
        self._connected = False

        self._serial_task_queue = None
        self.reactor.unregister_timer(self.serial_timer)

    def _calc_crc(self, buffer):
        _crc = 0xffff
        for byte in buffer:
            data = byte
            data ^= _crc & 0xff
            data ^= (data & 0x0f) << 4
            _crc = ((data << 8) | (_crc >> 8)) ^ (data >> 4) ^ (data << 3)
        return _crc

    def _update_and_get_request_id(self):
        if self._request_id >= 16382:
            self._request_id = 0
        else:
            self._request_id += 1

        return self._request_id


    def _write_serial(self, request):
        if not 'id' in request:
            request['id'] = self._update_and_get_request_id()

        payload = json.dumps(request)
        payload = bytes(payload, 'utf-8')

        data = bytes([0xFF, 0xAA])
        data += struct.pack('@H', len(payload))
        data += payload
        data += struct.pack('@H', self._calc_crc(payload))
        data += bytes([0xFE])

        # now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        # logging.info(f'[ACE] {now} >>> {request}')

        try:
            self._serial.write(data)
        except Exception as e:
            self.gcode.respond_info(f'[ACE] serial write exception {e}')
            return False

        return True

    def _reconnect_serial(self):
        if self._connected:
            self.gcode.respond_warn('[ACE] reconnect warning: serial port already connected')
            return True

        try:
            if self._serial != None and self._serial.isOpen():
                self._serial.close()
                self._connected = False

            self._serial = serial.Serial(port=self.serial_name,
                                        baudrate=self.baud)
            if self._serial.isOpen():
                self._connected = True

                if self._feed_assist_channel != -1:
                    self._enable_feed_assist(self._feed_assist_channel)
                self.gcode.respond_info('[ACE] Reconnected successfully.')
                return True
        except Exception as e:
            logging.warning(f'[ACE] reconnect error: {e}')

        return False
    
    def general_cb(self, response):
        if 'code' not in response or response['code'] != 0:
            raise ValueError('ACE Error: ' + response['msg'])
        
        self._info = response['result']

    def _send_heartbeat(self, id):
        self._callback_map[id] = self.general_cb
        if not self._write_serial({'id': id, 'method': 'get_status'}):
            return False

        return True

    def _writer(self):
        id = self._update_and_get_request_id()

        if not self._serial_task_queue.empty():
            task = self._serial_task_queue.peek()
            if task is not None:
                task[0]['id'] = id
                self._callback_map[id] = task[1]

                if not self._write_serial(task[0]):
                    if not task[2]:
                        # Not Retry
                        self._serial_task_queue.get()

                    return None

                self._serial_task_queue.get()

        else:
            if not self._send_heartbeat(id):
                return None

        return id

    def _reader(self):
        data = None

        for i in range(0, 2):
            try:
                data = self._serial.read_until(expected=bytes([0xFE]), size=4096)
            except Exception as e:
                self.gcode.respond_info(f'[ACE] read exception {e}')
                return None

            if None != data and len(data) >= 7:
                break

        if None == data or len(data) < 7:
            logging.info(f'[ACE] Read Too short')
            return None

        if data[0:2] != b'\xFF\xAA':
            logging.info(f'[ACE] Read invalid header')
            return None

        payload_length = struct.unpack('@H', data[2:4])[0]
        payload = data[4 : 4 + payload_length]
        # crc_received = data[4 + payload_length : 4 + payload_length + 2]

        # calculated_crc = self._calc_crc(payload)
        # if calculated_crc != crc_received:
        #     logging.info(f'[ACE] Read invalid CRC')
        #     return None

        try:
            json_str = payload.decode('utf-8')
            ret = json.loads(json_str)
        except Exception as e:
            logging.info(f'[ACE] Read invalid JSON')
            return None

        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        logging.info(f'[ACE] {now} <<< {ret}')
        id = ret['id']
        if id in self._callback_map:
            callback = self._callback_map.pop(id)
            if callback != None:
                callback(response=ret)

        return id

    def _serial_read_write(self, eventtime):
        if self._connected:
            send_id = self._writer()
            if None == send_id:
                self._connected = False
                return eventtime + 1

            read_id = self._reader()
            if read_id != send_id:
                self._connected = False
                return eventtime + 1
        else:
            self._reconnect_serial()
            return eventtime + 1

        return eventtime + 0.25


    def wait_ace_ready(self):
        while self._info['status'] != 'ready':
            self.dwell(delay=0.5)


    def send_request(self, request, callback, with_retry=True):
        self._serial_task_queue.put([request, callback, with_retry])


    def dwell(self, delay=1.):
        self.toolhead.dwell(delay)

    def _extruder_move(self, length, speed):
        pos = self.toolhead.get_position()
        pos[3] += length
        self.toolhead.move(pos, speed)
        return pos[3]

    def _create_mmu_sensor(self, config, pin, name):
        section = 'filament_switch_sensor %s' % name
        config.fileconfig.add_section(section)
        config.fileconfig.set(section, 'switch_pin', pin)
        config.fileconfig.set(section, 'pause_on_runout', 'False')
        fs = self.printer.load_object(config, section)

    def _feed(self, index, length, speed):
        self.send_request(
            request = {'method': 'feed_filament', 'params': {'index': index, 'length': length, 'speed': speed}},
            callback = self.general_cb
        )
        self.dwell((length / speed) + 0.1)

    def _retract(self, index, length, speed):
        self.send_request(
            request={'method': 'unwind_filament', 'params': {'index': index, 'length': length, 'speed': speed}},
            callback = self.general_cb
        )
        self.dwell(delay=(length / speed) + 0.1)

    def _enable_feed_assist(self, index):
        def callback(self, response):
            if 'code' in response and response['code'] != 0:
                raise ValueError('ACE Error: ' + response['msg'])
            else:
                self._feed_assist_channel = index
                self.gcode.respond_info('Enable ACE feed assist')

        self.send_request(request = {'method': 'start_feed_assist', 'params': {'index': index}}, callback=callback)
        self.dwell(0.7)

    def _disable_feed_assist(self, index):
        def callback(self, response):
            if 'code' in response and response['code'] != 0:
                raise self.gcode.error('ACE Error: ' + response['msg'])

            self._feed_assist_channel = -1
            self.gcode.respond_info('Disabled ACE feed assist')

        self.send_request(request = {'method': 'stop_feed_assist', 'params': {'index': index}}, callback=callback)
        self.dwell(0.3)

    def _park_to_toolhead(self, tool):
        sensor_extruder = self.printer.lookup_object('filament_switch_sensor extruder_sensor', None)
        sensor_toolhead = self.printer.lookup_object('filament_switch_sensor toolhead_sensor', None)

        self._enable_feed_assist(tool)

        while not bool(sensor_extruder.runout_helper.filament_present):
            self.dwell(delay=0.1)

        if not bool(sensor_extruder.runout_helper.filament_present):
            raise ValueError('Filament stuck ' + str(bool(sensor_extruder.runout_helper.filament_present)))
        else:
            self.variables['ace_filament_pos'] = 'spliter'

        while not bool(sensor_toolhead.runout_helper.filament_present):
            self._extruder_move(1, 5)

        self.variables['ace_filament_pos'] = 'toolhead'

        # The nozzle should be cleaned by brushing
        self.variables['ace_filament_pos'] = 'nozzle'

    def _reject_tool(self, index):
        self.gcode.respond_info(f'ACE: reject tool {index}')
        sensor_extruder = self.printer.lookup_object(f'filament_switch_sensor extruder_sensor', None)

        self._disable_feed_assist(index)
        self.wait_ace_ready()

        if  self.variables.get('ace_filament_pos', 'spliter') == 'nozzle':
            self.gcode.respond_info(f'ACE: cut tool {index}')
            self.gcode.run_script_from_command('CUT_TIP')
            self.variables['ace_filament_pos'] = 'toolhead'

        if  self.variables.get('ace_filament_pos', 'spliter') == 'toolhead':
            self.gcode.respond_info(f'ACE: extract tool {index} out of the extruder')
            while bool(sensor_extruder.runout_helper.filament_present):
                self._extruder_move(-20, 5)
                self._retract(index, 20, self.retract_speed)
                self.dwell(0.5)
            self.variables['ace_filament_pos'] = 'bowden'

        self.wait_ace_ready()

        self.gcode.respond_info(f'ACE: extract tool {index} out of the hub')
        self._retract(index, self.toolchange_retract_length, self.retract_speed)
        self.variables['ace_filament_pos'] = 'spliter'

        self.wait_ace_ready()

        self.gcode.respond_info(f'ACE: set current index -1')
        self.variables['ace_current_index'] = -1

    cmd_ACE_GET_CUR_INDEX_help = 'Get current tool index'
    def cmd_ACE_GET_CUR_INDEX(self, gcmd):
        self.gcode.respond_info('ACE Current index {}'.format(self.variables['ace_current_index']))

    cmd_ACE_START_DRYING_help = 'Starts ACE Pro dryer'
    def cmd_ACE_START_DRYING(self, gcmd):
        temperature = gcmd.get_int('TEMP')
        duration = gcmd.get_int('DURATION', 240)

        if duration <= 0:
            raise gcmd.error('Wrong duration')
        if temperature <= 0 or temperature > self.max_dryer_temperature:
            raise gcmd.error('Wrong temperature')

        def callback(self, response):
            if 'code' in response and response['code'] != 0:
                raise gcmd.error('ACE Error: ' + response['msg'])

            self.gcode.respond_info('Started ACE drying')

        self.send_request(request = {'method': 'drying', 'params': {'temp':temperature, 'fan_speed': 7000, 'duration': duration}}, callback=callback)


    cmd_ACE_STOP_DRYING_help = 'Stops ACE Pro dryer'
    def cmd_ACE_STOP_DRYING(self, gcmd):
        def callback(self, response):
            if 'code' in response and response['code'] != 0:
                raise gcmd.error('ACE Error: ' + response['msg'])

            self.gcode.respond_info('Stopped ACE drying')

        self.send_request(request = {'method':'drying_stop'}, callback=callback)

    cmd_ACE_ENABLE_FEED_ASSIST_help = 'Enables ACE feed assist'
    def cmd_ACE_ENABLE_FEED_ASSIST(self, gcmd):
        index = gcmd.get_int('INDEX')

        if index < 0 or index >= 4:
            raise gcmd.error('Wrong index')

        self._enable_feed_assist(index)

    cmd_ACE_DISABLE_FEED_ASSIST_help = 'Disables ACE feed assist'
    def cmd_ACE_DISABLE_FEED_ASSIST(self, gcmd):
        if self._feed_assist_channel != -1:
            index = gcmd.get_int('INDEX', self._feed_assist_channel)
        else:
            index = gcmd.get_int('INDEX')

        if index < 0 or index >= 4:
            raise gcmd.error('Wrong index')

        self._disable_feed_assist(index)

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

    cmd_ACE_CLEAR_ALL_STATUS_help = 'Clean status'
    def cmd_ACE_CLEAR_ALL_STATUS(self, gcmd):
        self.variables['ace_current_index'] = -1
        self.variables['ace_filament_pos'] = 'spliter'

    cmd_ACE_REJECT_TOOL_help = 'Reject tool'
    def cmd_ACE_REJECT_TOOL(self, gcmd):
        tool = gcmd.get_int('TOOL', -1)

        if -1 == tool:
            tool = self.variables.get('ace_current_index', -1)
        if tool != -1:
            self._reject_tool(tool)

    cmd_ACE_CHANGE_TOOL_help = 'Changes tool'
    def cmd_ACE_CHANGE_TOOL(self, gcmd):
        # self.gcode.respond_info('ACE: Changing tool...')
        tool = gcmd.get_int('TOOL')

        if tool < -1 or tool >= 4:
            raise gcmd.error('Wrong tool')

        was = self.variables.get('ace_current_index', -1)
        if was == tool:
            gcmd.respond_info('ACE: Not changing tool, current index already ' + str(tool))

            if -1 == self._feed_assist_channel:
                self._enable_feed_assist(tool)
            return

        if tool != -1:
            status = self._info['slots'][tool]['status']
            if status != 'ready':
                self.gcode.run_script_from_command('_ACE_ON_EMPTY_ERROR INDEX=' + str(tool))
                return

        self.gcode.run_script_from_command('_ACE_PRE_TOOLCHANGE FROM=' + str(was) + ' TO=' + str(tool))

        logging.info('ACE: Toolchange ' + str(was) + ' => ' + str(tool))
        if was != -1:
            self._reject_tool(was)

        if tool != -1:
            self._feed(tool, self.toolchange_retract_length-5, self.retract_speed)
            self.variables['ace_filament_pos'] = 'bowden'
            self.wait_ace_ready()

            self._park_to_toolhead(tool)

        self.gcode.run_script_from_command('_ACE_POST_TOOLCHANGE FROM=' + str(was) + ' TO=' + str(tool))

        self.variables['ace_current_index'] = tool
        # Force save to disk
        self.gcode.run_script_from_command('SAVE_VARIABLE VARIABLE=ace_current_index VALUE=' + str(tool))
        self.gcode.run_script_from_command(f"""SAVE_VARIABLE VARIABLE=ace_filament_pos VALUE='"{self.variables['ace_filament_pos']}"'""")

        gcmd.respond_info(f'Tool {tool} load')


    cmd_ACE_FILAMENT_STATUS_help = 'ACE Filament status'
    def cmd_ACE_FILAMENT_STATUS(self, gcmd):
        sensor_extruder = self.printer.lookup_object('filament_switch_sensor %s' % 'extruder_sensor', None)
        sensor_toolhead = self.printer.lookup_object('filament_switch_sensor %s' % 'toolhead_sensor', None)
        state = 'ACE----------|*--|Ex--|*----|Nz--'
        if  self.variables['ace_filament_pos'] == 'nozzle':
            state = 'ACE>>>>>>>>>>|*>>|Ex>>|*>>|Nz>>'
        if  self.variables['ace_filament_pos'] == 'toolhead' and bool(sensor_toolhead.runout_helper.filament_present):
            state = 'ACE>>>>>>>>>>|*>>|Ex>>|*>>|Nz--'
        if  self.variables['ace_filament_pos'] == 'toolhead' and not bool(sensor_toolhead.runout_helper.filament_present):
            state = 'ACE>>>>>>>>>>|*>>|Ex>>|*--|Nz--'
        if  self.variables['ace_filament_pos'] == 'bowden' and bool(sensor_extruder.runout_helper.filament_present):
            state = 'ACE>>>>>>>>>>|*>>|Ex--|*--|Nz--'
        if  self.variables['ace_filament_pos'] == 'bowden' and not bool(sensor_extruder.runout_helper.filament_present):
            state = 'ACE>>>>>>>>>>|*--|Ex--|*--|Nz--'
        gcmd.respond_info(state)

    cmd_ACE_DEBUG_help = 'ACE Debug'
    def cmd_ACE_DEBUG(self, gcmd):
        method = gcmd.get('METHOD')
        params = gcmd.get('PARAMS', '{}')

        try:
            def callback(self, response):
                self.gcode.respond_info(str(response))

            self.send_request(request = {'method': method, 'params': json.loads(params)}, callback=callback)
        except Exception as e:
            self.gcode.respond_info('Error: ' + str(e))


def load_config(config):
    return DuckAce(config)
