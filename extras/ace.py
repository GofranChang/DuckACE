import serial, time, logging, json, struct, queue, traceback
from datetime import datetime

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

        self._last_get_ace_response_time = None

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
            'ACE_REJECT_TOOL', self.cmd_ACE_REJECT_TOOL,
            desc=self.cmd_ACE_REJECT_TOOL_help)
        self.gcode.register_command(
            'ACE_CHANGE_TOOL', self.cmd_ACE_CHANGE_TOOL,
            desc=self.cmd_ACE_CHANGE_TOOL_help)
        self.gcode.register_command(
            'ACE_FILAMENT_STATUS', self.cmd_ACE_FILAMENT_STATUS,
            desc=self.cmd_ACE_FILAMENT_STATUS_help)
        self.gcode.register_command(
            'ACE_CLEAR_ALL_STATUS', self.cmd_ACE_CLEAR_ALL_STATUS,
            desc=self.cmd_ACE_CLEAR_ALL_STATUS_help)
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


    def _update_request_id(self):
        if self._request_id >= 16382:
            # self._reconnect_serial()
            self._request_id = 0
        else:
            self._request_id += 1

    def _send_with_retry(self, data, retry_times=3):
        for i in range(0, retry_times):
            try:
                self._serial.write(data)
                return
            except serial.SerialException as e:
                self.gcode.respond_info(f'[ACE] serial send exception {e}')
                self._reconnect_serial()
                self.pause(2 ** i)

    def _read_with_retry(self, expected, size, retry_times=3):
        for i in range(0, retry_times):
            try:
                ret = self._serial.read_until(expected=expected, size=size)
                return ret
            except serial.SerialException as e:
                self.gcode.respond_info(f'[ACE] serial read exception {e}')
                self._reconnect_serial()

        return None

    def _send_request(self, request, with_retry=True):
        if not 'id' in request:
            request['id'] = self._request_id
            self._update_request_id()

        payload = json.dumps(request)
        payload = bytes(payload, 'utf-8')

        data = bytes([0xFF, 0xAA])
        data += struct.pack('@H', len(payload))
        data += payload
        data += struct.pack('@H', self._calc_crc(payload))
        data += bytes([0xFE])

        now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        # logging.info(f'[ACE] >>> {request}')
        self.gcode.respond_info(f'[ACE] {now} >>> {request}')

        if with_retry:
            self._send_with_retry(data)
        else:
            self._serial.write(data)


    def _main_eval(self, eventtime):
        while not self._main_queue.empty():
            task = self._main_queue.get_nowait()
            if task is not None:
                task()

        return eventtime + 0.25

    def _reconnect_serial(self, max_attempts=3, delay=1):
        self._serial.close()

        for attempt in range(max_attempts):
            try:
                logging.info(f'Attempt {attempt+1} to reconnect...')
                self._serial = serial.Serial(port=self.serial_name,
                                            baudrate=self.baud)
                if self._serial.isOpen():
                    logging.info('Reconnected successfully.')
                    return
            except Exception as e:
                logging.info(f'Reconnect attempt {attempt+1} failed: {e}')


    def _reader(self):
        try:
            now = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            # self.gcode.respond_info(f'[ACE] {now} isconnected')
            if None != self._last_get_ace_response_time and time.time() - self._last_get_ace_response_time > 2:
                self.gcode.respond_info(f'[ACE] {self._last_get_ace_response_time} {time.time()}')
                self._reconnect_serial()

            # self.gcode.respond_info(f'[ACE] not timeout')
            ret = self._read_with_retry(expected=bytes([0xFE]), size=4096)
            # ret = self._serial.read_until(expected=([0xFE]), size=4096)
            if None == ret:
                return

            # self.gcode.respond_info(f'[ACE] readed')

            if not (ret[0] == 0xFF and ret[1] == 0xAA and ret[len(ret) - 1] == 0xFE):
                self.gcode.respond_info('ACE: Invalid data recieved: ' + str(ret))
                return

            rlen = struct.unpack('@H', ret[2:4])[0]
            crc_data = None
            crc_offset = 0
            if rlen > len(ret) - 7 or rlen <= 0:
                if rlen == len(ret) - 6:
                    crc_data = self._read_with_retry(expected=bytes([0xFE]), size=1)
                    # crc_data = self._serial.read_until(expected=([0xFE]), size=1)
                    crc_data = bytes([ret[len(ret) - 2], crc_data[0]])
                    ret = ret[0:len(ret) - 2] + bytes([ret[len(ret) - 1]])
                    crc_offset = 2
                elif rlen == len(ret) - 5:
                    crc_data = self._read_with_retry(expected=bytes([0xFE]), size=2)
                    if None == crc_data or len(crc_data) < 2:
                        return
                    # crc_data = self._serial.read_until(expected=([0xFE]), size=2)
                    # self.gcode.respond_info(f'XXXXXXXXXXXXXXXXXXXX {len(crc_data)}')
                    crc_data = bytes([crc_data[1], crc_data[0]])
                    crc_offset = 2
                else:
                    logging.info('ACE: Invalid data length recieved: ' + str(rlen) + ' | ' + str(len(ret)) + ', ' + str(ret))
                    return

            if crc_data is None:
                crc_data = ret[len(ret) - 3:len(ret) - 1]

            rpayload = ret[4:(len(ret) - 3 + crc_offset)]
            crc = struct.pack('@H', self._calc_crc(rpayload))
            if crc[0] != crc_data[0] or crc[1] != crc_data[1]:
                logging.info('ACE: Invalid data CRC recieved: ' + str(ret) + ', should be: ' + str(crc))
                return

            ret = json.loads(rpayload.decode('utf-8'))

            # self._last_get_ace_response_time = time.time()
            self.gcode.respond_info(f'[ACE] {now} <<< {ret}')
            id = ret['id']
            if id in self._callback_map:
                callback = self._callback_map.pop(id)
                callback(self = self, response = ret)
        except serial.serialutil.SerialException:
            self._printer.invoke_shutdown(f'Lost communication with ACE "{self._name}"')
            return
        except Exception as e:
            logging.info('ACE: Read error ' + traceback.format_exc(e))


    def _writer(self):
        try:
            while not self._queue.empty():
                task = self._queue.get()
                if task is not None:
                    id = self._request_id
                    self._update_request_id()
                    self._callback_map[id] = task[1]
                    task[0]['id'] = id

                    self._send_request(task[0], task[2])

            def callback(self, response):
                if response is not None:
                    self._info = response['result']
                    # logging.info('ACE: Update status ' + str(self._request_id))

                    if self._park_in_progress and self._info['status'] == 'ready':
                        new_assist_count = self._info['feed_assist_count']
                        if new_assist_count > self._last_assist_count:
                            self._last_assist_count = new_assist_count
                            self.pause(0.7, True) # 0.68 + small room 0.02 for response
                            self._assist_hit_count = 0
                        elif self._assist_hit_count < self.park_hit_count:
                            self._assist_hit_count += 1
                            self.pause(0.7, True)
                        else:
                            self._assist_hit_count = 0
                            self._park_in_progress = False
                            logging.info('ACE: Parked to toolhead with assist count: ' + str(self._last_assist_count))

                            if self._park_is_toolchange:
                                self._park_is_toolchange = False
                                def main_callback():
                                    self.gcode.run_script_from_command('_ACE_POST_TOOLCHANGE FROM=' + str(self._park_previous_tool) + ' TO=' + str(self._park_index))
                                self._main_queue.put(main_callback)
                            else:
                                self._send_request({'method': 'stop_feed_assist', 'params': {'index': self._park_index}})

            id = self._request_id
            self._update_request_id()
            self._callback_map[id] = callback

            self._send_request({'id': id, 'method': 'get_status'}, with_retry=False)
        except serial.serialutil.SerialException:
            self._printer.invoke_shutdown(f'Lost communication with ACE "{self._name}"')
        except Exception as e:
            logging.info('ACE: Write error ' + str(e))

    def _serial_read_write(self, eventtime):
        if self._connected:
            self._writer()
            self._reader()

        if self._park_in_progress:
            next_time = 0.68
        else:
            next_time = 0.5
        return eventtime + next_time


    def _handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')

        logging.info('ACE: Connecting to ' + self.serial_name)

        # We can catch timing where ACE reboots itself when no data is available from host. We're avoiding it with this hack
        self._connected = False
        for i in range(0, 10):
            try:
                self._serial = serial.Serial(
                    port          = self.serial_name,
                    baudrate      = self.baud)

                if self._serial.isOpen():
                    self._connected = True
                    break
            except serial.serialutil.SerialException:
                time.sleep(0.5)
                continue

        if not self._connected:
            raise ValueError('ACE: Failed to connect to ' + self.serial_name)

        logging.info('ACE: Connected to ' + self.serial_name)

        self._queue = queue.Queue()
        self._main_queue = queue.Queue()

        self.serial_timer = self.reactor.register_timer(self._serial_read_write, self.reactor.NOW)

        self.main_timer = self.reactor.register_timer(self._main_eval, self.reactor.NOW)

        def info_callback(self, response):
            res = response['result']
            self.gcode.respond_info('Connected ' + res['model'] + ' ' + res['firmware'])
        self.send_request(request = {'method': 'get_info'}, callback = info_callback)


    def _handle_disconnect(self):
        logging.info('ACE: Closing connection to ' + self.serial_name)
        self._serial.close()
        self._connected = False
        self.reactor.unregister_timer(self.main_timer)
        self.reactor.unregister_timer(self.serial_timer)

        self._queue = None
        self._main_queue = None

    def wait_ace_ready(self):
        while self._info['status'] != 'ready':
            self.pause(delay=0.5)


    def send_request(self, request, callback, with_retry=True):
        self._queue.put([request, callback, with_retry])


    def pause(self, delay = 1., on_main = False):
        def main_callback():
            event_time = self.reactor.monotonic()
            self.reactor.pause(event_time + delay)

        if on_main:
            self._main_queue.put(main_callback)
        else:
            main_callback()

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

        self.send_request(request = {'method': 'drying', 'params': {'temp':temperature, 'fan_speed': 7000, 'duration': duration}}, callback = callback)


    cmd_ACE_STOP_DRYING_help = 'Stops ACE Pro dryer'
    def cmd_ACE_STOP_DRYING(self, gcmd):
        def callback(self, response):
            if 'code' in response and response['code'] != 0:
                raise gcmd.error('ACE Error: ' + response['msg'])

            self.gcode.respond_info('Stopped ACE drying')

        self.send_request(request = {'method':'drying_stop'}, callback = callback)

    def _enable_feed_assist(self, index):
        def callback(self, response):
            if 'code' in response and response['code'] != 0:
                raise ValueError('ACE Error: ' + response['msg'])
            else:
                self._feed_assist_index = index
                self.gcode.respond_info(str(response))

        self.send_request(request = {'method': 'start_feed_assist', 'params': {'index': index}}, callback = callback)
        self.pause(delay = 0.7)

    cmd_ACE_ENABLE_FEED_ASSIST_help = 'Enables ACE feed assist'
    def cmd_ACE_ENABLE_FEED_ASSIST(self, gcmd):
        index = gcmd.get_int('INDEX')

        if index < 0 or index >= 4:
            raise gcmd.error('Wrong index')

        self._enable_feed_assist(index)


    def _disable_feed_assist(self, index):
        def callback(self, response):
            if 'code' in response and response['code'] != 0:
                raise self.gcode.error('ACE Error: ' + response['msg'])

            self._feed_assist_index = -1
            self.gcode.respond_info('Disabled ACE feed assist')

        self.send_request(request = {'method': 'stop_feed_assist', 'params': {'index': index}}, callback = callback)
        self.pause(0.3)

    cmd_ACE_DISABLE_FEED_ASSIST_help = 'Disables ACE feed assist'
    def cmd_ACE_DISABLE_FEED_ASSIST(self, gcmd):
        if self._feed_assist_index != -1:
            index = gcmd.get_int('INDEX', self._feed_assist_index)
        else:
            index = gcmd.get_int('INDEX')

        if index < 0 or index >= 4:
            raise gcmd.error('Wrong index')

        self._disable_feed_assist(index)



    def _feed(self, index, length, speed):
        def callback(self, response):
            if 'code' in response and response['code'] != 0:
                raise ValueError('ACE Error: ' + response['msg'])

        self.send_request(request = {'method': 'feed_filament', 'params': {'index': index, 'length': length, 'speed': speed}}, callback = callback)
        self.pause((length / speed) + 0.1)

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
        def callback(self, response):
            if 'code' in response and response['code'] != 0:
                raise ValueError('ACE Error: ' + response['msg'])

        self.send_request(
            request={'method': 'unwind_filament', 'params': {'index': index, 'length': length, 'speed': speed}},
            callback=callback)
        self.pause(delay=(length / speed) + 0.1)

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
        sensor_extruder = self.printer.lookup_object('filament_switch_sensor %s' % 'extruder_sensor', None)
        sensor_toolhead = self.printer.lookup_object('filament_switch_sensor %s' % 'toolhead_sensor', None)

        self._enable_feed_assist(tool)

        while not bool(sensor_extruder.runout_helper.filament_present):
            self.pause(delay=0.1)

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
        sensor_extruder = self.printer.lookup_object('filament_switch_sensor %s' % 'extruder_sensor', None)

        self._disable_feed_assist(index)
        self.wait_ace_ready()
        if  self.variables.get('ace_filament_pos', 'spliter') == 'nozzle':
            self.gcode.run_script_from_command('CUT_TIP')
            self.variables['ace_filament_pos'] = 'toolhead'

        if  self.variables.get('ace_filament_pos', 'spliter') == 'toolhead':
            while bool(sensor_extruder.runout_helper.filament_present):
                self._extruder_move(-20, 5)
                self._retract(index, 20, self.retract_speed)
                self.pause(1)
            self.variables['ace_filament_pos'] = 'bowden'

        self.wait_ace_ready()

        self._retract(index, self.toolchange_retract_length, self.retract_speed)
        self.variables['ace_filament_pos'] = 'spliter'

        self.wait_ace_ready()

        self.variables['ace_current_index'] = -1

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
        self.gcode.respond_info('ACE: Changing tool...')
        tool = gcmd.get_int('TOOL')

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
        self.gcode.run_script_from_command(f'''SAVE_VARIABLE VARIABLE=ace_filament_pos VALUE=''{self.variables['ace_filament_pos']}''''')

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

            self.send_request(request = {'method': method, 'params': json.loads(params)}, callback = callback)
        except Exception as e:
            self.gcode.respond_info('Error: ' + str(e))


def load_config(config):
    return DuckAce(config)
