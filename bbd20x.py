'''
    BBD20X Stage Driver for SRAS
    Thomas Ales | Feb 2026
    Version 2
'''

import time
from threading import Thread, Event
from queue import Queue, Empty
from apt_constants import StatusBits, TriggerBitsServo
from apt_messages import APTProtocol
from serial_comms import SerialSnooper


class ThorlabsServoDriver():
    # These are specific to the MLS203-1
    # change for a different application
    counts_per_mm = 20000
    accel_scaling = 13.744
    velocity_scaling = 134217.73

    def __init__(self):
        self.am_connected = False
        self.am_enabled = [False, False]
        self.am_homed = [False, False]
        self.am_moving = [False, False]
        self.am_error = [False, False]
        self.serial_port = "/dev/ttyUSB1"
        self.serial_spd = 115200
        self.max_velocities = [100.0, 100.0] # mm/s
        self.max_accels = [500.0, 500.0]     # mm/s2
        self.positions = [-1.0, -1.0]
        self.act_velocities = [-1.0, -1.0]
        self.current_demand = [0, 0]
        self.serial_snoop = None
        self.am_listening = False
        self.pending_responses = {}  # msg_id -> {'event': Event, 'data': None}
        self.bays_present = []  # list of bay addresses that responded
        self._tx_queue = Queue()
        self._polling_active = False
        self._poll_interval = 0.2  # seconds between poll cycles

    def connect(self, port=None, spd=None):
        if port is None:
            port = self.serial_port
        else:
            self.serial_port = port

        if spd is None:
            spd = self.serial_spd
        else:
            self.serial_spd = spd

        self.serial_snoop = SerialSnooper(port, spd)
        self.serial_snoop.start()
        self.am_listening = True

        # Start worker threads
        self._tx_thread = Thread(target=self._tx_worker, daemon=True)
        self._tx_thread.start()
        self._rx_thread = Thread(target=self._rx_worker, daemon=True)
        self._rx_thread.start()
        self._poll_thread = Thread(target=self._poll_worker, daemon=True)
        self._poll_thread.start()

        # Give threads time to start
        time.sleep(0.3)

        # Query which bays are present
        self.bays_present = []
        for bay_id in range(10):  # bays 0-9
            try:
                result = self.send_and_wait(0x0060, timeout=0.5,
                                            bay_id=bay_id,
                                            destination=0x11,
                                            source=0x01)
                if result and result.get('bay_state') == 0x01:
                    bay_addr = 0x21 + bay_id
                    self.bays_present.append(bay_addr)

            except TimeoutError:
                # Bay not present or not responding
                pass

        if not self.bays_present:
            print("  [WARN] No bays detected!")

        self.am_connected = True

    # ── Worker threads ───────────────────────────────────────────

    def _tx_worker(self):
        '''Single serial writer. All outbound messages flow through
           _tx_queue so there are no write races on the serial port.'''
        while self.am_listening:
            try:
                msg = self._tx_queue.get(timeout=0.05)
                self.serial_snoop.serial_connection.write(msg)
                self.serial_snoop.serial_connection.flush()
            except Empty:
                continue
            except (OSError, TypeError):
                break

    def _rx_worker(self):
        '''Listens for messages on the serial RX queue and dispatches them.'''
        while self.am_listening:
            try:
                _msg = self.serial_snoop.rx_msg_queue.get(timeout=0.05)

                # Try to parse the message, skip if unknown
                try:
                    _msgid, data = APTProtocol.unpack_message(_msg)
                except ValueError:
                    print(f"  [WARN] Unknown message: {_msg.hex()}")
                    continue

                # Check if someone is waiting for this response
                if _msgid in self.pending_responses:
                    self.pending_responses[_msgid]['data'] = data
                    self.pending_responses[_msgid]['event'].set()

                # Status update → ACK via TX queue, then update state
                if _msgid == 0x0491:
                    if self.am_listening:
                        _ack = APTProtocol.build_message(0x0492, source=0x01,
                                                         destination=_msg[5])
                        self._tx_queue.put(_ack)
                    self._update0x491(data)
                # Move completed → update state
                elif _msgid == 0x0464:
                    self._update0x0464(data)
            except Empty:
                continue

        return

    def _poll_worker(self):
        '''Periodically sends REQ_USTATUSUPDATE (0x0490) to each bay.
           The 0x0491 responses are ACKed by _rx_worker through the
           TX queue, which keeps the controller's comms watchdog alive.'''
        while self.am_listening:
            if self._polling_active:
                for axis_addr in (self.bays_present or [0x21, 0x22]):
                    if not self.am_listening:
                        break
                    msg = APTProtocol.build_message(0x0490, source=0x01,
                                                    destination=axis_addr)
                    self._tx_queue.put(msg)
                time.sleep(self._poll_interval)
            else:
                time.sleep(0.05)

    # ── Polling control ──────────────────────────────────────────

    def start_polling(self, interval=0.2):
        '''Start periodic status polling (interval in seconds).'''
        self._poll_interval = interval
        self._polling_active = True

    def stop_polling(self):
        '''Stop periodic status polling.'''
        self._polling_active = False

    # ── Core messaging ───────────────────────────────────────────

    def send_and_wait(self, msg_id, timeout=10.0, retries=1, **kwargs):
        """Send a message and wait for its expected response.
           On timeout, drains serial buffer and retries up to `retries` times."""
        msg_spec = APTProtocol.MSGS.get(msg_id)
        if not msg_spec:
            raise ValueError(f"Unknown message: {hex(msg_id)}")

        expected = msg_spec.get('response')
        msg = APTProtocol.build_message(msg_id, **kwargs)

        for attempt in range(1 + retries):
            # Set up listener before sending
            if expected:
                evt = Event()
                self.pending_responses[expected] = {'event': evt, 'data': None}

            self._tx_queue.put(msg)

            if not expected:
                return None  # no response expected

            # Wait for response
            if evt.wait(timeout=timeout):
                data = self.pending_responses[expected]['data']
                del self.pending_responses[expected]
                return data
            else:
                del self.pending_responses[expected]
                if attempt < retries:
                    # Drain serial input buffer and message queue, then retry
                    self.serial_snoop.serial_connection.reset_input_buffer()
                    time.sleep(0.05)
                    while not self.serial_snoop.rx_msg_queue.empty():
                        try:
                            self.serial_snoop.rx_msg_queue.get_nowait()
                        except Empty:
                            break
                    print(f"  [RETRY] {APTProtocol.get_name(msg_id)} attempt {attempt+2}")

        raise TimeoutError(f"Timeout waiting for {hex(expected)}")

    def send_message(self, msg_id, **kwargs):
        '''Build and queue a message for transmission (fire-and-forget).'''
        msg = APTProtocol.build_message(msg_id, **kwargs)
        self._tx_queue.put(msg)

    # ── Connection management ────────────────────────────────────

    def disconnect(self):
        if (self.serial_snoop and
            self.am_listening is True):
            # Stop polling first
            self._polling_active = False
            # Queue disconnect messages for the TX worker to send
            for addr in [0x11, 0x21, 0x22]:
                self._tx_queue.put(
                    APTProtocol.build_message(0x0002, destination=addr,
                                              source=0x01))
            time.sleep(0.2)  # let TX worker flush them out
            # Stop all worker loops, then wait for threads to exit
            self.am_listening = False
            self.serial_snoop.stop()
            self._rx_thread.join()
            self._tx_thread.join()
            self._poll_thread.join()
            self.serial_snoop.join()
            # Close port only after all threads are done
            self.serial_snoop.close()

    # ── State update handlers ────────────────────────────────────

    def _update0x491(self, msg):
        '''
            _update0x491 - Internal function for handling USTATUSUPDATE
            messages and updating the data for that particular axis sending
            the message.
        '''
        if msg['source'] == 0x21:
            ch = 0
        elif msg['source'] == 0x22:
            ch = 1
        else:
            return

        self.positions[ch] = msg['position'] / self.counts_per_mm
        self.act_velocities[ch] = msg['velocity'] / self.velocity_scaling
        self.current_demand[ch] = msg['motor_current']

        if(msg['status_bits'] & StatusBits.MOT_ANY_ERR):
            self.am_error[ch] = True
        else:
            self.am_error[ch] = False

        if(msg['status_bits'] & StatusBits.MOT_ANY_MOVE):
            self.am_moving[ch] = True
        else:
            self.am_moving[ch] = False

        if(msg['status_bits'] & StatusBits.MOT_SB_HOMED):
            self.am_homed[ch] = True

    def _update0x0464(self, msg):
        '''
            _update0x0464 - internal function for MOVE_COMPLETED messages.
            Updates position, velocity, and moving state.
        '''
        if msg['source'] == 0x21:
            ch = 0
        elif msg['source'] == 0x22:
            ch = 1
        else:
            return

        self.positions[ch] = msg['position'] / self.counts_per_mm
        self.act_velocities[ch] = msg['velocity'] / self.velocity_scaling
        self.current_demand[ch] = msg['motor_current']
        self.am_moving[ch] = False
        return

    def _update0x0212(self, msg):
        '''
            _update0x0212 - internal function that listens for CHANENABLESTATE
            messages.
        '''
        if msg['source'] == 0x21:
            ch = 0
        elif msg['source'] == 0x22:
            ch = 1
        else:
            raise ValueError("Wherever this message came from, it's WRONG!")

        if msg['enable_state'] == 0x01:
            self.am_enabled[ch] = True # enabled
        elif msg['enable_state'] == 0x02:
            self.am_enabled[ch] = False # disabled
        else:
            raise ValueError("Am I a joke to you? WTF did this even come from?!")

    # ── Axis control ─────────────────────────────────────────────

    def enable_axis(self, axis):
        '''Enable the specified axis (0x21 = X, 0x22 = Y).'''
        self.send_message(0x0210, chan_ident=1, enable_state=0x01,
                          destination=axis, source=0x01)

    def disable_axis(self, axis):
        '''Disable the specified axis (0x21 = X, 0x22 = Y).'''
        self.send_message(0x0210, chan_ident=1, enable_state=0x02,
                          destination=axis, source=0x01)

    def toggle_enabled_state(self, axis):
        '''
            toggle_enabled_state(axis) - enables the axis if disabled. disables
            if enabled. not much more to it.
        '''
        if axis == 0x21:
            ch = 0
        elif axis == 0x22:
            ch = 1
        else:
            raise ValueError("I don't know that axis!")
        # get the old state and flip it like a sample
        new_state = not self.am_enabled[ch]
        self.send_message(0x0210, chan_ident=1,
                          enable_state=0x01 if new_state else 0x02,
                          destination=axis, source=0x01)

    def home_axis(self, axis, timeout=60.0):
        '''
            home_axis(axis, timeout=60): Blocking home command. Required at
            power up. Default timeout is 60s, but 20-30s is fine as well if
            you're in that much of a hurry.
        '''
        if axis == 0x21:
            ch = 0
        elif axis == 0x22:
            ch = 1
        else:
            raise ValueError("I don't know that axis!")

        self.send_and_wait(0x0443, timeout=timeout, chan_ident=1,
                           destination=axis, source=0x01)
        return

    def move_axis_relative(self, axis, distance_in_mm, timeout=10.0):
        '''
            move_axis_relative(axis, distance_in_mm, timeout=10):
                moves the specified axis a specified distance in mm.
                Timeout defaults to ten seconds.
        '''
        # sanity check
        if axis == 0x21 and abs(distance_in_mm) > 110.0:
            raise ValueError("You can't move farther than the stage is long.")
        elif axis == 0x22 and abs(distance_in_mm) > 75.0:
            raise ValueError("You can't move farther than the stage is wide.")

        _distance_in_encoder = int(round(distance_in_mm * self.counts_per_mm))
        self.send_and_wait(0x0448, timeout=timeout, chan_ident=1,
                           relative_distance=_distance_in_encoder,
                           destination=axis, source=0x01)
        return

    def move_axis_absolute(self, axis, position_in_mm, timeout=10.0):
        '''
            move_axis_absolute(axis, position_in_mm, timeout=10):
                moves the specified axis to an absolute position in mm.
                Timeout defaults to ten seconds.
        '''
        if axis == 0x21 and (position_in_mm < 0.0 or position_in_mm > 110.0):
            raise ValueError("Position out of range for X axis (0-110 mm).")
        elif axis == 0x22 and (position_in_mm < 0.0 or position_in_mm > 75.0):
            raise ValueError("Position out of range for Y axis (0-75 mm).")

        _position_in_encoder = int(round(position_in_mm * self.counts_per_mm))
        self.send_and_wait(0x0453, timeout=timeout, chan_ident=1,
                           absolute_distance=_position_in_encoder,
                           destination=axis, source=0x01)
        return

    # ── Velocity parameters ──────────────────────────────────────

    def get_velocity_params(self, axis, timeout=5.0):
        '''
            get_velocity_params(axis): Queries the current velocity parameters
            for the specified axis. Returns a dict with keys:
                min_velocity (mm/s), acceleration (mm/s2), max_velocity (mm/s)
        '''
        if axis == 0x21:
            ch = 0
        elif axis == 0x22:
            ch = 1
        else:
            raise ValueError("I don't know that axis!")

        result = self.send_and_wait(0x0414, timeout=timeout, chan_ident=1,
                                    zero_this=0x00, destination=axis,
                                    source=0x01)
        params = {
            'min_velocity': result['min_velocity'] / self.velocity_scaling,
            'acceleration': result['acceleration'] / self.accel_scaling,
            'max_velocity': result['max_velocity'] / self.velocity_scaling,
        }

        self.max_velocities[ch] = params['max_velocity']
        self.max_accels[ch] = params['acceleration']

        return params

    def set_velocity_params(self, axis, max_velocity=None, acceleration=None):
        '''
            set_velocity_params(axis, max_velocity=None, acceleration=None):
                Sets velocity and/or acceleration for the specified axis.
                Values are in mm/s and mm/s2 respectively. Any parameter
                left as None keeps its current value.
        '''
        if axis == 0x21:
            ch = 0
        elif axis == 0x22:
            ch = 1
        else:
            raise ValueError("I don't know that axis!")

        # Only query current params if we need to fill in a missing value
        if max_velocity is None or acceleration is None:
            current = self.get_velocity_params(axis)
            if max_velocity is None:
                max_velocity = current['max_velocity']
            if acceleration is None:
                acceleration = current['acceleration']

        # Update the cached values
        self.max_velocities[ch] = max_velocity
        self.max_accels[ch] = acceleration

        _min_v = 0
        _accel = int(round(acceleration * self.accel_scaling))
        _max_v = int(round(max_velocity * self.velocity_scaling))

        self.send_message(0x0413, chan_ident=1,
                          min_velocity=_min_v,
                          acceleration=_accel,
                          max_velocity=_max_v,
                          destination=axis, source=0x01)

    # ── Trigger control ───────────────────────────────────────

    def set_trigger(self, axis, mode):
        '''
            set_trigger(axis, mode): Sets the trigger mode for the specified
            axis. Mode should be a TriggerBitsServo value or combination.
        '''
        self.send_message(0x0500, chan_ident=1, mode=int(mode),
                          destination=axis, source=0x01)

    def get_trigger(self, axis, timeout=5.0):
        '''
            get_trigger(axis): Queries the current trigger mode for the
            specified axis. Returns the mode byte as a TriggerBitsServo.
        '''
        result = self.send_and_wait(0x0501, timeout=timeout,
                                    chan_ident=1, mode=0x00,
                                    destination=axis, source=0x01)
        return TriggerBitsServo(result['mode'])

    def set_trigger_trigin_high(self, axis):
        '''Set trigger input to logic high.'''
        self.set_trigger(axis, TriggerBitsServo.TRIGIN_HIGH)

    def set_trigger_trigin_relmove(self, axis):
        '''Set trigger input to initiate a relative move.'''
        self.set_trigger(axis, TriggerBitsServo.TRIGIN_RELMOVE)

    def set_trigger_trigin_absmove(self, axis):
        '''Set trigger input to initiate an absolute move.'''
        self.set_trigger(axis, TriggerBitsServo.TRIGIN_ABSMOVE)

    def set_trigger_trigin_homemove(self, axis):
        '''Set trigger input to initiate a home move.'''
        self.set_trigger(axis, TriggerBitsServo.TRIGIN_HOMEMOVE)

    def set_trigger_trigout_high(self, axis):
        '''Set trigger output to logic high.'''
        self.set_trigger(axis, TriggerBitsServo.TRIGOUT_HIGH)

    def set_trigger_trigout_inmotion(self, axis):
        '''Set trigger output high while axis is in motion.'''
        self.set_trigger(axis, TriggerBitsServo.TRIGOUT_INMOTION)

    def set_trigger_trigout_motioncomplete(self, axis):
        '''Set trigger output to pulse when motion completes.'''
        self.set_trigger(axis, TriggerBitsServo.TRIGOUT_MOTIONCOMPLETE)

    def set_trigger_trigout_maxvelocity(self, axis):
        '''Set trigger output to pulse at max velocity.'''
        self.set_trigger(axis, TriggerBitsServo.TRIGOUT_MAXVELOCITY)

    def set_trigger_trigout_maxv(self, axis):
        '''Set trigger output high + pulse at max velocity (TRIGOUT_MAXV).'''
        self.set_trigger(axis, TriggerBitsServo.TRIGOUT_MAXV)
