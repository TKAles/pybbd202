'''
	MLS203/BBD202 Thorlabs Controller Driver
	Thomas Ales | Rev 0 | Jan 2026
	This does contain LLM generated code. Cause if you think 
	I am gonna copy 443 pages of the protocol manual by hand
	you're crazy.

	Only supports two axes, but easily extendable to 3 if you have a 203 controller
'''


from pyftdi.ftdi import Ftdi
import time
import struct
import threading
import queue
from typing import Union, Optional, Callable
from dataclasses import dataclass
from enum import IntEnum, IntFlag


class MsgId(IntEnum):
    """APT message IDs."""
    HW_DISCONNECT = 0x0002
    HW_REQ_INFO = 0x0005
    HW_GET_INFO = 0x0006
    HW_STOP_UPDATEMSGS = 0x0012
    HW_START_UPDATEMSGS = 0x0011
    MOD_IDENTIFY = 0x0223
    MOD_SET_CHANENABLESTATE = 0x0210
    MOD_REQ_CHANENABLESTATE = 0x0211
    MOD_GET_CHANENABLESTATE = 0x0212
    MOT_MOVE_HOME = 0x0443
    MOT_MOVE_HOMED = 0x0444
    MOT_MOVE_ABSOLUTE = 0x0453
    MOT_MOVE_COMPLETED = 0x0464
    MOT_REQ_USTATUSUPDATE = 0x0490
    MOT_GET_USTATUSUPDATE = 0x0491
    MOT_GET_DCSTATUSUPDATE = 0x0491  # Alias for USTATUSUPDATE
    MOT_ACK_USTATUSUPDATE = 0x0492
    MOT_SET_ENCCOUNTER = 0x0409
    MOT_REQ_ENCCOUNTER = 0x040A
    MOT_GET_ENCCOUNTER = 0x040B
    MOT_REQ_POSCOUNTER = 0x0411
    MOT_GET_POSCOUNTER = 0x0412
    MOT_SET_VELPARAMS = 0x0413
    MOT_REQ_VELPARAMS = 0x0414
    MOT_GET_VELPARAMS = 0x0415
    MOT_SET_JOGPARAMS = 0x0416
    MOT_REQ_JOGPARAMS = 0x0417
    MOT_GET_JOGPARAMS = 0x0418
    MOT_SET_MOVERELPARAMS = 0x0445
    MOT_REQ_MOVERELPARAMS = 0x0446
    MOT_GET_MOVERELPARAMS = 0x0447
    MOT_MOVE_RELATIVE = 0x0448
    MOT_SET_MOVEABSPARAMS = 0x0450
    MOT_REQ_MOVEABSPARAMS = 0x0451
    MOT_GET_MOVEABSPARAMS = 0x0452
    MOT_MOVE_STOP = 0x0465
    MOT_MOVE_STOPPED = 0x0466


class ChannelEnableState(IntEnum):
    """Channel enable state values."""
    DISABLED = 0x02
    ENABLED = 0x01


class JogMode(IntEnum):
    """Jog mode values."""
    CONTINUOUS = 0x01
    SINGLE_STEP = 0x02


class StopMode(IntEnum):
    """Stop mode values."""
    IMMEDIATE = 0x01
    CONTROLLED = 0x02


class MotorStatusBits(IntFlag):
    """Motor status bit flags."""
    CWHARDLIMIT = 0x00000001        # Clockwise hard limit triggered
    CCWHARDLIMIT = 0x00000002       # Counter-clockwise hard limit triggered
    CWSOFTLIMIT = 0x00000004        # Clockwise software limit triggered
    CCWSOFTLIMIT = 0x00000008       # Counter-clockwise software limit triggered
    INMOTIONCW = 0x00000010         # Stage is in motion clockwise
    INMOTIONCCW = 0x00000020        # Stage is in motion counter-clockwise
    CONNECTED = 0x00000100          # Motor recognized by controller
    HOMING = 0x00000200             # Motor is homing
    HOMED = 0x00000400              # Motor completed homing, position valid
    INITIALIZING = 0x00000800       # Motor is performing phase initialization
    TRACKING = 0x00001000           # Position is within tracking window
    SETTLED = 0x00002000            # Motor not moving and settled at target
    POSITIONERROR = 0x00004000      # Actual position outside tracking window
    OVERTEMP = 0x00200000           # Overtemperature error
    BUSVOLTFAULT = 0x00400000       # Supply voltage too low
    COMMUTATIONERROR = 0x00800000   # Motor commutation error (power cycle required)
    OVERLOAD = 0x01000000           # Motor overload/overcurrent condition
    ENCODERFAULT = 0x02000000       # Encoder error
    OVERCURRENT = 0x04000000        # Continuous current limit exceeded
    POWEROK = 0x10000000            # Power supply is OK
    ACTIVE = 0x20000000             # Controller executing motion command
    ERROR = 0x40000000              # Other errors
    ENABLED = 0x80000000            # Motor output enabled, maintaining position


@dataclass
class AptMessage:
    """Represents a parsed APT message."""
    msg_id: int
    param1: int
    param2: int
    dest: int
    source: int
    data: bytes
    raw: bytes
    timestamp: float


class MotionController:
    """Controller class for Thorlabs APT motion controllers via FTDI interface."""

    TYPE_FORMATS = {
        'word': '<H',
        'short': '<h',
        'dword': '<I',
        'long': '<l',
        'char': '<B',
    }

    # Channel/axis destination addresses
    DEST_CONTROLLER = 0x11  # Motherboard/controller
    DEST_X_AXIS = 0x21
    DEST_Y_AXIS = 0x22

    # Scaling factors for BBD202/203
    ENCODER_COUNTS_PER_MM = 20000  # Position scaling
    VELOCITY_SCALING = 13421.77    # Velocity (mm/s) scaling
    ACCELERATION_SCALING = 13.744  # Acceleration (mm/s^2) scaling

    # Class variable to track if FTDI product is registered
    _ftdi_registered = False

    def __init__(self, url: str = 'ftdi://0x0403:0xfaf0/1', baudrate: int = 115200):
        # Only register the custom product once globally
        if not MotionController._ftdi_registered:
            try:
                Ftdi.add_custom_product(0x0403, 0xfaf0, 'Thorlabs')
                MotionController._ftdi_registered = True
            except ValueError:
                # Already registered, ignore
                pass

        self.ftdi = Ftdi()
        self.url = url
        self.baudrate = baudrate
        self._connected = False
        self._hw_info = None

        self._encoder_counts = {}  # dest -> encoder count (int)
        self._stage_positions = {}  # dest -> position in mm (float)
        self._position_lock = threading.Lock()

        self._velocity_params = {}  # dest -> {'min_velocity': float, 'acceleration': float, 'max_velocity': float}
        self._velocity_lock = threading.Lock()

        self._jog_params = {}  # dest -> {'jog_mode': int, 'step_size': int, 'min_velocity': float, 'acceleration': float, 'max_velocity': float, 'stop_mode': int}
        self._jog_lock = threading.Lock()

        self._move_rel_params = {}  # dest -> {'relative_distance': float, 'relative_distance_counts': int}
        self._move_rel_lock = threading.Lock()

        self._move_abs_params = {}  # dest -> {'absolute_position': float, 'absolute_position_counts': int}
        self._move_abs_lock = threading.Lock()

        self._move_completed = {}  # dest -> {'position': float, 'position_counts': int, 'velocity': int, 'motor_current': int, 'status_bits': int}
        self._move_completed_lock = threading.Lock()

        self._move_stopped = {}  # dest -> {'position': float, 'position_counts': int, 'velocity': int, 'motor_current': int, 'status_bits': int}
        self._move_stopped_lock = threading.Lock()

        self._status_bits = {}  # dest -> int (updated on every MOVE_COMPLETED or MOVE_STOPPED)
        self._status_bits_lock = threading.Lock()

        # CRITICAL: Controller stops responding after ~50 commands without periodic ACK
        self._ack_thread: Optional[threading.Thread] = None
        self._ack_running = False

        self._rx_thread: Optional[threading.Thread] = None
        self._rx_running = False
        self._rx_queue: queue.Queue[AptMessage] = queue.Queue()
        self._rx_lock = threading.Lock()

        self._callbacks: dict[int, list[Callable[[AptMessage], None]]] = {}

        self._waiters: dict[int, tuple[threading.Event, list]] = {}
        self._waiter_lock = threading.Lock()
    
    def connect(self, enable_updates: bool = True) -> None:
        """
        Open connection to the controller.
        
        Args:
            enable_updates: If True, enable status update messages and start listener thread
        """
        self.ftdi.open_from_url(self.url)
        self.ftdi.set_baudrate(self.baudrate)
        self.ftdi.set_line_property(8, 1, 'N')
        self.ftdi.set_flowctrl('hw')
        self.ftdi.set_rts(True)
        self.ftdi.purge_buffers()
        self._connected = True
        time.sleep(0.1)

        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        if enable_updates:
            self._send_raw(self._build_short_message(MsgId.HW_START_UPDATEMSGS))
        else:
            self._send_raw(self._build_short_message(MsgId.HW_STOP_UPDATEMSGS))

        time.sleep(0.1)
    
    def disconnect(self) -> None:
        """Close connection to the controller."""
        if self._connected:
            self.stop_status_ack()

            self._rx_running = False
            if self._rx_thread:
                self._rx_thread.join(timeout=1.0)
                self._rx_thread = None

            try:
                self._send_raw(self._build_short_message(MsgId.HW_DISCONNECT))
                time.sleep(0.05)
            except:
                pass

            self.ftdi.close()
            self._connected = False
            self._hw_info = None
    
    def _build_short_message(self, msg_id: int, param1: int = 0, param2: int = 0, 
                              dest: int = 0x11, source: int = 0x01) -> bytes:
        """Build a 6-byte APT header-only message."""
        return struct.pack('<HBBBB', msg_id, param1, param2, dest, source)
    
    def _build_long_message(self, msg_id: int, data: bytes, 
                             dest: int = 0x11, source: int = 0x01) -> bytes:
        """Build an APT message with data payload."""
        data_len = len(data)
        header = struct.pack('<HHBB', msg_id, data_len, dest | 0x80, source)
        return header + data
    
    def _send_raw(self, data: bytes) -> int:
        """Send raw bytes to the controller."""
        if not self._connected:
            raise ConnectionError("Not connected to controller")
        with self._rx_lock:
            return self.ftdi.write_data(data)
    
    def _rx_loop(self) -> None:
        """Receiver thread loop - continuously reads and parses messages."""
        buffer = b''
        
        while self._rx_running:
            try:
                with self._rx_lock:
                    chunk = self.ftdi.read_data(512)
                
                if chunk:
                    buffer += chunk
                    buffer = self._parse_buffer(buffer)
                else:
                    time.sleep(0.01)
            except Exception as e:
                if self._rx_running:
                    print(f"RX error: {e}")
                time.sleep(0.1)
    
    def _parse_buffer(self, buffer: bytes) -> bytes:
        """Parse complete messages from buffer, return remaining bytes."""
        while len(buffer) >= 6:
            msg_id = struct.unpack('<H', buffer[0:2])[0]

            if buffer[4] & 0x80:
                # Long message: bytes 2-3 are data length
                data_len = struct.unpack('<H', buffer[2:4])[0]
                total_len = 6 + data_len

                if len(buffer) < total_len:
                    break

                dest = buffer[4] & 0x7F
                source = buffer[5]
                data = buffer[6:total_len]
                raw = buffer[:total_len]

                msg = AptMessage(
                    msg_id=msg_id,
                    param1=0,
                    param2=0,
                    dest=dest,
                    source=source,
                    data=data,
                    raw=raw,
                    timestamp=time.time()
                )
            else:
                # Short message: bytes 2-3 are param1, param2
                param1 = buffer[2]
                param2 = buffer[3]
                dest = buffer[4]
                source = buffer[5]

                msg = AptMessage(
                    msg_id=msg_id,
                    param1=param1,
                    param2=param2,
                    dest=dest,
                    source=source,
                    data=b'',
                    raw=buffer[:6],
                    timestamp=time.time()
                )
                total_len = 6
            
            buffer = buffer[total_len:]

            self._rx_queue.put(msg)

            with self._waiter_lock:
                if msg.msg_id in self._waiters:
                    event, responses = self._waiters[msg.msg_id]
                    responses.append(msg)
                    event.set()

                # Also check for composite keys (msg_id, source)
                # GET messages always have dest=0x01 (us)
                if msg.msg_id in (MsgId.MOD_GET_CHANENABLESTATE, MsgId.HW_GET_INFO, MsgId.MOT_GET_ENCCOUNTER, MsgId.MOT_GET_VELPARAMS, MsgId.MOT_GET_JOGPARAMS, MsgId.MOT_GET_MOVERELPARAMS, MsgId.MOT_GET_MOVEABSPARAMS):
                    if msg.dest != 0x01:
                        print(f"Warning: GET message {msg.msg_id:04X} has unexpected dest 0x{msg.dest:02X}, expected 0x01")

                composite_key = (msg.msg_id, msg.source)
                if composite_key in self._waiters:
                    event, responses = self._waiters[composite_key]
                    responses.append(msg)
                    event.set()
            
            if msg.msg_id == MsgId.MOT_GET_ENCCOUNTER:
                if len(msg.raw) >= 12:
                    encoder_counts = struct.unpack('<i', msg.raw[8:12])[0]
                    position_mm = encoder_counts / self.ENCODER_COUNTS_PER_MM
                    with self._position_lock:
                        self._encoder_counts[msg.source] = encoder_counts
                        self._stage_positions[msg.source] = position_mm

            # MOT_MOVE_HOMED is a short message - positions auto-reset to 0
            if msg.msg_id == MsgId.MOT_MOVE_HOMED:
                with self._position_lock:
                    self._encoder_counts[msg.source] = 0
                    self._stage_positions[msg.source] = 0.0

            if msg.msg_id == MsgId.MOT_GET_VELPARAMS:
                if len(msg.raw) >= 20:
                    min_vel_counts = struct.unpack('<l', msg.raw[8:12])[0]
                    accel_counts = struct.unpack('<L', msg.raw[12:16])[0]
                    max_vel_counts = struct.unpack('<L', msg.raw[16:20])[0]

                    min_velocity = min_vel_counts / self.VELOCITY_SCALING
                    acceleration = accel_counts / self.ACCELERATION_SCALING
                    max_velocity = max_vel_counts / self.VELOCITY_SCALING

                    with self._velocity_lock:
                        self._velocity_params[msg.source] = {
                            'min_velocity': min_velocity,
                            'acceleration': acceleration,
                            'max_velocity': max_velocity
                        }

            if msg.msg_id == MsgId.MOT_GET_JOGPARAMS:
                if len(msg.raw) >= 28:
                    jog_mode = struct.unpack('<H', msg.raw[8:10])[0]
                    step_size_counts = struct.unpack('<L', msg.raw[10:14])[0]
                    min_vel_counts = struct.unpack('<L', msg.raw[14:18])[0]
                    accel_counts = struct.unpack('<L', msg.raw[18:22])[0]
                    max_vel_counts = struct.unpack('<L', msg.raw[22:26])[0]
                    stop_mode = struct.unpack('<H', msg.raw[26:28])[0]

                    step_size_mm = step_size_counts / self.ENCODER_COUNTS_PER_MM
                    min_velocity = min_vel_counts / self.VELOCITY_SCALING
                    acceleration = accel_counts / self.ACCELERATION_SCALING
                    max_velocity = max_vel_counts / self.VELOCITY_SCALING

                    with self._jog_lock:
                        self._jog_params[msg.source] = {
                            'jog_mode': jog_mode,
                            'step_size': step_size_mm,
                            'step_size_counts': step_size_counts,
                            'min_velocity': min_velocity,
                            'acceleration': acceleration,
                            'max_velocity': max_velocity,
                            'stop_mode': stop_mode
                        }

            if msg.msg_id == MsgId.MOT_GET_MOVERELPARAMS:
                if len(msg.raw) >= 12:
                    relative_distance_counts = struct.unpack('<i', msg.raw[8:12])[0]
                    relative_distance_mm = relative_distance_counts / self.ENCODER_COUNTS_PER_MM

                    with self._move_rel_lock:
                        self._move_rel_params[msg.source] = {
                            'relative_distance': relative_distance_mm,
                            'relative_distance_counts': relative_distance_counts
                        }

            if msg.msg_id == MsgId.MOT_GET_MOVEABSPARAMS:
                if len(msg.raw) >= 12:
                    absolute_position_counts = struct.unpack('<i', msg.raw[8:12])[0]
                    absolute_position_mm = absolute_position_counts / self.ENCODER_COUNTS_PER_MM

                    with self._move_abs_lock:
                        self._move_abs_params[msg.source] = {
                            'absolute_position': absolute_position_mm,
                            'absolute_position_counts': absolute_position_counts
                        }

            if msg.msg_id == MsgId.MOT_MOVE_COMPLETED:
                if len(msg.raw) >= 20:
                    position_counts = struct.unpack('<i', msg.raw[8:12])[0]
                    velocity = struct.unpack('<H', msg.raw[12:14])[0]
                    motor_current = struct.unpack('<h', msg.raw[14:16])[0]
                    status_bits = struct.unpack('<I', msg.raw[16:20])[0]

                    position_mm = position_counts / self.ENCODER_COUNTS_PER_MM

                    with self._move_completed_lock:
                        self._move_completed[msg.source] = {
                            'position': position_mm,
                            'position_counts': position_counts,
                            'velocity': velocity,
                            'motor_current': motor_current,
                            'status_bits': status_bits
                        }

                    with self._status_bits_lock:
                        self._status_bits[msg.source] = status_bits

            if msg.msg_id == MsgId.MOT_MOVE_STOPPED:
                if len(msg.raw) >= 20:
                    position_counts = struct.unpack('<i', msg.raw[8:12])[0]
                    velocity = struct.unpack('<H', msg.raw[12:14])[0]
                    motor_current = struct.unpack('<h', msg.raw[14:16])[0]
                    status_bits = struct.unpack('<I', msg.raw[16:20])[0]

                    position_mm = position_counts / self.ENCODER_COUNTS_PER_MM

                    with self._move_stopped_lock:
                        self._move_stopped[msg.source] = {
                            'position': position_mm,
                            'position_counts': position_counts,
                            'velocity': velocity,
                            'motor_current': motor_current,
                            'status_bits': status_bits
                        }

                    with self._status_bits_lock:
                        self._status_bits[msg.source] = status_bits

            # Call registered callbacks
            if msg.msg_id in self._callbacks:
                for callback in self._callbacks[msg.msg_id]:
                    try:
                        callback(msg)
                    except Exception as e:
                        print(f"Callback error: {e}")
        
        return buffer
    
    def register_callback(self, msg_id: int, callback: Callable[[AptMessage], None]) -> None:
        """
        Register a callback for a specific message ID.
        
        Args:
            msg_id: Message ID to listen for
            callback: Function to call when message is received
        """
        if msg_id not in self._callbacks:
            self._callbacks[msg_id] = []
        self._callbacks[msg_id].append(callback)
    
    def unregister_callback(self, msg_id: int, callback: Callable[[AptMessage], None]) -> None:
        """Remove a callback for a specific message ID."""
        if msg_id in self._callbacks:
            self._callbacks[msg_id].remove(callback)
    
    def wait_for_message(self, msg_id: int, timeout: float = 5.0) -> Optional[AptMessage]:
        """
        Wait for a specific message ID.
        
        Args:
            msg_id: Message ID to wait for
            timeout: Timeout in seconds
            
        Returns:
            The received message, or None if timeout
        """
        event = threading.Event()
        responses = []
        
        with self._waiter_lock:
            self._waiters[msg_id] = (event, responses)
        
        try:
            if event.wait(timeout=timeout):
                return responses[0] if responses else None
            return None
        finally:
            with self._waiter_lock:
                del self._waiters[msg_id]
    
    def send_command(self, msg_id: int, param1: int = 0, param2: int = 0,
                     data: Optional[bytes] = None, dest: int = 0x11, source: int = 0x01) -> None:
        """
        Send a command to the controller.
        
        Args:
            msg_id: Message ID
            param1: Parameter 1 (for short messages)
            param2: Parameter 2 (for short messages)
            data: Data payload (for long messages)
            dest: Destination address
            source: Source address
        """
        if data is not None:
            msg = self._build_long_message(msg_id, data, dest, source)
        else:
            msg = self._build_short_message(msg_id, param1, param2, dest, source)
        self._send_raw(msg)
    
    def send_and_wait(self, msg_id: int, response_id: int, param1: int = 0, param2: int = 0,
                      data: Optional[bytes] = None, timeout: float = 5.0) -> Optional[AptMessage]:
        """
        Send a command and wait for a response.
        
        Args:
            msg_id: Message ID to send
            response_id: Message ID to wait for
            param1: Parameter 1 (for short messages)
            param2: Parameter 2 (for short messages)
            data: Data payload (for long messages)
            timeout: Timeout in seconds
            
        Returns:
            The response message, or None if timeout
        """
        event = threading.Event()
        responses = []
        
        with self._waiter_lock:
            self._waiters[response_id] = (event, responses)
        
        try:
            self.send_command(msg_id, param1, param2, data)
            if event.wait(timeout=timeout):
                return responses[0] if responses else None
            return None
        finally:
            with self._waiter_lock:
                del self._waiters[response_id]
    
    def get_message(self, timeout: float = 0.1) -> Optional[AptMessage]:
        """
        Get the next message from the receive queue.
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            The next message, or None if queue is empty
        """
        try:
            return self._rx_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def get_all_messages(self) -> list[AptMessage]:
        """Get all messages currently in the receive queue."""
        messages = []
        while True:
            try:
                messages.append(self._rx_queue.get_nowait())
            except queue.Empty:
                break
        return messages
    
    def convert(self, data: bytes, dtype: str) -> Union[int, str]:
        """
        Convert a byte slice to the specified type.
        
        Args:
            data: Byte slice to convert
            dtype: Type specification:
                   - 'word': unsigned 16-bit integer
                   - 'short': signed 16-bit integer (2's complement)
                   - 'dword': unsigned 32-bit integer
                   - 'long': signed 32-bit integer (2's complement)
                   - 'char': single byte
                   - 'char[n]': string of n characters (e.g., 'char[8]')
                   
        Returns:
            Converted value (int for numeric types, str for char[n])
        """
        if dtype.startswith('char[') and dtype.endswith(']'):
            n = int(dtype[5:-1])
            if len(data) < n:
                raise ValueError(f"Data length {len(data)} < expected {n} for {dtype}")
            return data[:n].decode('ascii').rstrip('\x00')
        
        if dtype not in self.TYPE_FORMATS:
            raise ValueError(f"Unknown type: {dtype}")
        
        fmt = self.TYPE_FORMATS[dtype]
        expected_size = struct.calcsize(fmt)
        
        if len(data) < expected_size:
            raise ValueError(f"Data length {len(data)} < expected {expected_size} for {dtype}")
        
        return struct.unpack(fmt, data[:expected_size])[0]
    
    # Hardware info methods
    
    def _refresh_hw_info(self) -> None:
        """Fetch and cache hardware info from the controller."""
        response = self.send_and_wait(MsgId.HW_REQ_INFO, MsgId.HW_GET_INFO, timeout=2.0)
        
        if response is None:
            raise IOError("No response to HW_REQ_INFO")
        
        if len(response.raw) < 90:
            raise IOError(f"Invalid HW_GET_INFO response: expected 90 bytes, got {len(response.raw)}")
        
        self._hw_info = response.raw
    
    def get_serial_number(self) -> int:
        """Get the controller serial number."""
        if self._hw_info is None:
            self._refresh_hw_info()
        return self.convert(self._hw_info[6:10], 'dword')
    
    def get_model(self) -> str:
        """Get the controller model name."""
        if self._hw_info is None:
            self._refresh_hw_info()
        return self.convert(self._hw_info[10:18], 'char[8]')
    
    def get_hw_type(self) -> int:
        """Get the hardware type identifier."""
        if self._hw_info is None:
            self._refresh_hw_info()
        return self.convert(self._hw_info[18:20], 'word')
    
    def get_firmware_version(self) -> str:
        """Get the firmware version as a string (major.interim.minor)."""
        if self._hw_info is None:
            self._refresh_hw_info()
        
        fw_minor = self.convert(self._hw_info[20:21], 'char')
        fw_interim = self.convert(self._hw_info[21:22], 'char')
        fw_major = self.convert(self._hw_info[22:23], 'char')
        
        return f"{fw_major}.{fw_interim}.{fw_minor}"
    
    def get_firmware_version_tuple(self) -> tuple:
        """Get the firmware version as a tuple (major, interim, minor)."""
        if self._hw_info is None:
            self._refresh_hw_info()
        
        fw_minor = self.convert(self._hw_info[20:21], 'char')
        fw_interim = self.convert(self._hw_info[21:22], 'char')
        fw_major = self.convert(self._hw_info[22:23], 'char')
        
        return (fw_major, fw_interim, fw_minor)
    
    def get_notes(self) -> str:
        """Get the controller notes/description string."""
        if self._hw_info is None:
            self._refresh_hw_info()
        return self.convert(self._hw_info[24:84], 'char[60]')
    
    def get_hw_version(self) -> int:
        """Get the hardware version number."""
        if self._hw_info is None:
            self._refresh_hw_info()
        return self.convert(self._hw_info[84:86], 'word')
    
    def get_mod_state(self) -> int:
        """Get the module state."""
        if self._hw_info is None:
            self._refresh_hw_info()
        return self.convert(self._hw_info[86:88], 'word')
    
    def get_num_channels(self) -> int:
        """Get the number of channels."""
        if self._hw_info is None:
            self._refresh_hw_info()
        return self.convert(self._hw_info[88:90], 'word')
    
    def get_hw_info(self) -> dict:
        """
        Get all hardware info as a dictionary.
        
        Returns:
            Dictionary containing all hardware info fields
        """
        if self._hw_info is None:
            self._refresh_hw_info()
        
        return {
            'serial_number': self.get_serial_number(),
            'model': self.get_model(),
            'hw_type': self.get_hw_type(),
            'firmware_version': self.get_firmware_version(),
            'notes': self.get_notes(),
            'hw_version': self.get_hw_version(),
            'mod_state': self.get_mod_state(),
            'num_channels': self.get_num_channels(),
        }
    
    # Position methods

    def get_position(self, dest: int, timeout: float = 5.0) -> Optional[float]:
        """
        Get the current position of an axis by sending REQ and waiting for GET response.

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            timeout: Timeout in seconds

        Returns:
            The position in mm as a float, or None if timeout

        Raises:
            ValueError: If dest is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x21 (X-axis) or 0x22 (Y-axis)")

        # Create a unique waiter key combining message ID and expected source
        waiter_key = (MsgId.MOT_GET_ENCCOUNTER, dest)
        event = threading.Event()
        responses = []

        with self._waiter_lock:
            self._waiters[waiter_key] = (event, responses)

        try:
            # Send REQ message to the axis, device will respond with GET
            self.send_command(
                MsgId.MOT_REQ_ENCCOUNTER,
                param1=0x01,
                param2=0x00,
                dest=dest,  # Send to the axis
                source=0x01  # From us
            )

            # Wait for GET response from device
            if event.wait(timeout=timeout):
                response = responses[0] if responses else None
                if response is None:
                    return None
                # Bytes 8-11 of message contain encoder counts as 32-bit signed little-endian integer
                if len(response.raw) >= 12:
                    encoder_counts = struct.unpack('<i', response.raw[8:12])[0]
                    position_mm = encoder_counts / self.ENCODER_COUNTS_PER_MM
                    return position_mm
            return None
        finally:
            with self._waiter_lock:
                if waiter_key in self._waiters:
                    del self._waiters[waiter_key]
    
    def get_stage_position_x(self) -> Optional[float]:
        """
        Get the current cached position of the X-axis.

        Returns:
            The position in mm as a float, or None if not yet known
        """
        with self._position_lock:
            return self._stage_positions.get(self.DEST_X_AXIS)

    def get_stage_position_y(self) -> Optional[float]:
        """
        Get the current cached position of the Y-axis.

        Returns:
            The position in mm as a float, or None if not yet known
        """
        with self._position_lock:
            return self._stage_positions.get(self.DEST_Y_AXIS)

    def get_stage_positions(self) -> dict:
        """
        Get all cached stage positions.

        Returns:
            Dictionary mapping axis destination (0x21, 0x22) to position in mm
        """
        with self._position_lock:
            return dict(self._stage_positions)

    # Encoder count properties

    @property
    def encoder_count_x(self) -> Optional[int]:
        """
        Get the current cached encoder count of the X-axis.

        Returns:
            The encoder count as an int, or None if not yet known
        """
        with self._position_lock:
            return self._encoder_counts.get(self.DEST_X_AXIS)

    @property
    def encoder_count_y(self) -> Optional[int]:
        """
        Get the current cached encoder count of the Y-axis.

        Returns:
            The encoder count as an int, or None if not yet known
        """
        with self._position_lock:
            return self._encoder_counts.get(self.DEST_Y_AXIS)

    @property
    def position_x(self) -> Optional[float]:
        """
        Get the current cached position of the X-axis in mm.

        Returns:
            The position in mm as a float, or None if not yet known
        """
        with self._position_lock:
            return self._stage_positions.get(self.DEST_X_AXIS)

    @property
    def position_y(self) -> Optional[float]:
        """
        Get the current cached position of the Y-axis in mm.

        Returns:
            The position in mm as a float, or None if not yet known
        """
        with self._position_lock:
            return self._stage_positions.get(self.DEST_Y_AXIS)

    # Velocity parameter properties

    @property
    def velocity_params_x(self) -> Optional[dict]:
        """
        Get the current cached velocity parameters for the X-axis.

        Returns:
            Dictionary with 'min_velocity', 'acceleration', 'max_velocity' (mm/s or mm/s²),
            or None if not yet known
        """
        with self._velocity_lock:
            params = self._velocity_params.get(self.DEST_X_AXIS)
            return dict(params) if params else None

    @property
    def velocity_params_y(self) -> Optional[dict]:
        """
        Get the current cached velocity parameters for the Y-axis.

        Returns:
            Dictionary with 'min_velocity', 'acceleration', 'max_velocity' (mm/s or mm/s²),
            or None if not yet known
        """
        with self._velocity_lock:
            params = self._velocity_params.get(self.DEST_Y_AXIS)
            return dict(params) if params else None

    @property
    def min_velocity_x(self) -> Optional[float]:
        """Get the cached minimum velocity for X-axis in mm/s."""
        with self._velocity_lock:
            params = self._velocity_params.get(self.DEST_X_AXIS)
            return params['min_velocity'] if params else None

    @property
    def min_velocity_y(self) -> Optional[float]:
        """Get the cached minimum velocity for Y-axis in mm/s."""
        with self._velocity_lock:
            params = self._velocity_params.get(self.DEST_Y_AXIS)
            return params['min_velocity'] if params else None

    @property
    def acceleration_x(self) -> Optional[float]:
        """Get the cached acceleration for X-axis in mm/s²."""
        with self._velocity_lock:
            params = self._velocity_params.get(self.DEST_X_AXIS)
            return params['acceleration'] if params else None

    @property
    def acceleration_y(self) -> Optional[float]:
        """Get the cached acceleration for Y-axis in mm/s²."""
        with self._velocity_lock:
            params = self._velocity_params.get(self.DEST_Y_AXIS)
            return params['acceleration'] if params else None

    @property
    def max_velocity_x(self) -> Optional[float]:
        """Get the cached maximum velocity for X-axis in mm/s."""
        with self._velocity_lock:
            params = self._velocity_params.get(self.DEST_X_AXIS)
            return params['max_velocity'] if params else None

    @property
    def max_velocity_y(self) -> Optional[float]:
        """Get the cached maximum velocity for Y-axis in mm/s."""
        with self._velocity_lock:
            params = self._velocity_params.get(self.DEST_Y_AXIS)
            return params['max_velocity'] if params else None

    # Jog parameter properties

    @property
    def jog_params_x(self) -> Optional[dict]:
        """
        Get the current cached jog parameters for the X-axis.

        Returns:
            Dictionary with 'jog_mode', 'step_size', 'min_velocity', 'acceleration',
            'max_velocity', 'stop_mode', or None if not yet known
        """
        with self._jog_lock:
            params = self._jog_params.get(self.DEST_X_AXIS)
            return dict(params) if params else None

    @property
    def jog_params_y(self) -> Optional[dict]:
        """
        Get the current cached jog parameters for the Y-axis.

        Returns:
            Dictionary with 'jog_mode', 'step_size', 'min_velocity', 'acceleration',
            'max_velocity', 'stop_mode', or None if not yet known
        """
        with self._jog_lock:
            params = self._jog_params.get(self.DEST_Y_AXIS)
            return dict(params) if params else None

    # Move relative parameter properties

    @property
    def move_rel_params_x(self) -> Optional[dict]:
        """
        Get the current cached move relative parameters for the X-axis.

        Returns:
            Dictionary with 'relative_distance' (mm) and 'relative_distance_counts',
            or None if not yet known
        """
        with self._move_rel_lock:
            params = self._move_rel_params.get(self.DEST_X_AXIS)
            return dict(params) if params else None

    @property
    def move_rel_params_y(self) -> Optional[dict]:
        """
        Get the current cached move relative parameters for the Y-axis.

        Returns:
            Dictionary with 'relative_distance' (mm) and 'relative_distance_counts',
            or None if not yet known
        """
        with self._move_rel_lock:
            params = self._move_rel_params.get(self.DEST_Y_AXIS)
            return dict(params) if params else None

    @property
    def relative_distance_x(self) -> Optional[float]:
        """Get the cached relative move distance for X-axis in mm."""
        with self._move_rel_lock:
            params = self._move_rel_params.get(self.DEST_X_AXIS)
            return params['relative_distance'] if params else None

    @property
    def relative_distance_y(self) -> Optional[float]:
        """Get the cached relative move distance for Y-axis in mm."""
        with self._move_rel_lock:
            params = self._move_rel_params.get(self.DEST_Y_AXIS)
            return params['relative_distance'] if params else None

    # Move absolute parameter properties

    @property
    def move_abs_params_x(self) -> Optional[dict]:
        """
        Get the current cached move absolute parameters for the X-axis.

        Returns:
            Dictionary with 'absolute_position' (mm) and 'absolute_position_counts',
            or None if not yet known
        """
        with self._move_abs_lock:
            params = self._move_abs_params.get(self.DEST_X_AXIS)
            return dict(params) if params else None

    @property
    def move_abs_params_y(self) -> Optional[dict]:
        """
        Get the current cached move absolute parameters for the Y-axis.

        Returns:
            Dictionary with 'absolute_position' (mm) and 'absolute_position_counts',
            or None if not yet known
        """
        with self._move_abs_lock:
            params = self._move_abs_params.get(self.DEST_Y_AXIS)
            return dict(params) if params else None

    @property
    def absolute_position_x(self) -> Optional[float]:
        """Get the cached absolute move position for X-axis in mm."""
        with self._move_abs_lock:
            params = self._move_abs_params.get(self.DEST_X_AXIS)
            return params['absolute_position'] if params else None

    @property
    def absolute_position_y(self) -> Optional[float]:
        """Get the cached absolute move position for Y-axis in mm."""
        with self._move_abs_lock:
            params = self._move_abs_params.get(self.DEST_Y_AXIS)
            return params['absolute_position'] if params else None

    # Move completed status properties

    @property
    def move_completed_x(self) -> Optional[dict]:
        """
        Get the most recent move completion status for the X-axis.

        Returns:
            Dictionary with 'position' (mm), 'position_counts', 'velocity',
            'motor_current', 'status_bits', or None if no move completed yet
        """
        with self._move_completed_lock:
            status = self._move_completed.get(self.DEST_X_AXIS)
            return dict(status) if status else None

    @property
    def move_completed_y(self) -> Optional[dict]:
        """
        Get the most recent move completion status for the Y-axis.

        Returns:
            Dictionary with 'position' (mm), 'position_counts', 'velocity',
            'motor_current', 'status_bits', or None if no move completed yet
        """
        with self._move_completed_lock:
            status = self._move_completed.get(self.DEST_Y_AXIS)
            return dict(status) if status else None

    # Move stopped status properties

    @property
    def move_stopped_x(self) -> Optional[dict]:
        """
        Get the most recent move stopped status for the X-axis.

        Returns:
            Dictionary with 'position' (mm), 'position_counts', 'velocity',
            'motor_current', 'status_bits', or None if no move stopped event yet
        """
        with self._move_stopped_lock:
            status = self._move_stopped.get(self.DEST_X_AXIS)
            return dict(status) if status else None

    @property
    def move_stopped_y(self) -> Optional[dict]:
        """
        Get the most recent move stopped status for the Y-axis.

        Returns:
            Dictionary with 'position' (mm), 'position_counts', 'velocity',
            'motor_current', 'status_bits', or None if no move stopped event yet
        """
        with self._move_stopped_lock:
            status = self._move_stopped.get(self.DEST_Y_AXIS)
            return dict(status) if status else None

    # Status bits properties

    @property
    def status_bits_x(self) -> Optional[int]:
        """
        Get the current status bits for the X-axis.

        Returns:
            Current status bits as int, or None if not yet known
        """
        with self._status_bits_lock:
            return self._status_bits.get(self.DEST_X_AXIS)

    @property
    def status_bits_y(self) -> Optional[int]:
        """
        Get the current status bits for the Y-axis.

        Returns:
            Current status bits as int, or None if not yet known
        """
        with self._status_bits_lock:
            return self._status_bits.get(self.DEST_Y_AXIS)

    # X-axis status bit properties

    @property
    def is_enabled_x(self) -> bool:
        """Check if X-axis motor output is enabled."""
        bits = self.status_bits_x
        return bool(bits and (MotorStatusBits.ENABLED in MotorStatusBits(bits)))

    @property
    def is_homed_x(self) -> bool:
        """Check if X-axis has been homed (position count is valid)."""
        bits = self.status_bits_x
        return bool(bits and (MotorStatusBits.HOMED in MotorStatusBits(bits)))

    @property
    def is_homing_x(self) -> bool:
        """Check if X-axis is currently homing."""
        bits = self.status_bits_x
        return bool(bits and (MotorStatusBits.HOMING in MotorStatusBits(bits)))

    @property
    def is_in_motion_x(self) -> bool:
        """Check if X-axis is currently in motion."""
        bits = self.status_bits_x
        return bool(bits and self.is_in_motion(bits))

    @property
    def is_settled_x(self) -> bool:
        """Check if X-axis is settled at target position."""
        bits = self.status_bits_x
        return bool(bits and (MotorStatusBits.SETTLED in MotorStatusBits(bits)))

    @property
    def is_tracking_x(self) -> bool:
        """Check if X-axis position is within tracking window."""
        bits = self.status_bits_x
        return bool(bits and (MotorStatusBits.TRACKING in MotorStatusBits(bits)))

    @property
    def is_connected_x(self) -> bool:
        """Check if X-axis motor is recognized by controller."""
        bits = self.status_bits_x
        return bool(bits and (MotorStatusBits.CONNECTED in MotorStatusBits(bits)))

    @property
    def has_errors_x(self) -> bool:
        """Check if X-axis has any error conditions."""
        bits = self.status_bits_x
        return bool(bits and self.has_errors(bits))

    @property
    def power_ok_x(self) -> bool:
        """Check if X-axis power supply is OK."""
        bits = self.status_bits_x
        return bool(bits and (MotorStatusBits.POWEROK in MotorStatusBits(bits)))

    @property
    def is_active_x(self) -> bool:
        """Check if X-axis controller is executing a motion command."""
        bits = self.status_bits_x
        return bool(bits and (MotorStatusBits.ACTIVE in MotorStatusBits(bits)))

    @property
    def at_cw_limit_x(self) -> bool:
        """Check if X-axis is at clockwise limit (hard or soft)."""
        bits = self.status_bits_x
        if not bits:
            return False
        status = MotorStatusBits(bits)
        return bool((MotorStatusBits.CWHARDLIMIT in status) or (MotorStatusBits.CWSOFTLIMIT in status))

    @property
    def at_ccw_limit_x(self) -> bool:
        """Check if X-axis is at counter-clockwise limit (hard or soft)."""
        bits = self.status_bits_x
        if not bits:
            return False
        status = MotorStatusBits(bits)
        return bool((MotorStatusBits.CCWHARDLIMIT in status) or (MotorStatusBits.CCWSOFTLIMIT in status))

    # Y-axis status bit properties

    @property
    def is_enabled_y(self) -> bool:
        """Check if Y-axis motor output is enabled."""
        bits = self.status_bits_y
        return bool(bits and (MotorStatusBits.ENABLED in MotorStatusBits(bits)))

    @property
    def is_homed_y(self) -> bool:
        """Check if Y-axis has been homed (position count is valid)."""
        bits = self.status_bits_y
        return bool(bits and (MotorStatusBits.HOMED in MotorStatusBits(bits)))

    @property
    def is_homing_y(self) -> bool:
        """Check if Y-axis is currently homing."""
        bits = self.status_bits_y
        return bool(bits and (MotorStatusBits.HOMING in MotorStatusBits(bits)))

    @property
    def is_in_motion_y(self) -> bool:
        """Check if Y-axis is currently in motion."""
        bits = self.status_bits_y
        return bool(bits and self.is_in_motion(bits))

    @property
    def is_settled_y(self) -> bool:
        """Check if Y-axis is settled at target position."""
        bits = self.status_bits_y
        return bool(bits and (MotorStatusBits.SETTLED in MotorStatusBits(bits)))

    @property
    def is_tracking_y(self) -> bool:
        """Check if Y-axis position is within tracking window."""
        bits = self.status_bits_y
        return bool(bits and (MotorStatusBits.TRACKING in MotorStatusBits(bits)))

    @property
    def is_connected_y(self) -> bool:
        """Check if Y-axis motor is recognized by controller."""
        bits = self.status_bits_y
        return bool(bits and (MotorStatusBits.CONNECTED in MotorStatusBits(bits)))

    @property
    def has_errors_y(self) -> bool:
        """Check if Y-axis has any error conditions."""
        bits = self.status_bits_y
        return bool(bits and self.has_errors(bits))

    @property
    def power_ok_y(self) -> bool:
        """Check if Y-axis power supply is OK."""
        bits = self.status_bits_y
        return bool(bits and (MotorStatusBits.POWEROK in MotorStatusBits(bits)))

    @property
    def is_active_y(self) -> bool:
        """Check if Y-axis controller is executing a motion command."""
        bits = self.status_bits_y
        return bool(bits and (MotorStatusBits.ACTIVE in MotorStatusBits(bits)))

    @property
    def at_cw_limit_y(self) -> bool:
        """Check if Y-axis is at clockwise limit (hard or soft)."""
        bits = self.status_bits_y
        if not bits:
            return False
        status = MotorStatusBits(bits)
        return bool((MotorStatusBits.CWHARDLIMIT in status) or (MotorStatusBits.CWSOFTLIMIT in status))

    @property
    def at_ccw_limit_y(self) -> bool:
        """Check if Y-axis is at counter-clockwise limit (hard or soft)."""
        bits = self.status_bits_y
        if not bits:
            return False
        status = MotorStatusBits(bits)
        return bool((MotorStatusBits.CCWHARDLIMIT in status) or (MotorStatusBits.CCWSOFTLIMIT in status))

    def start_update_messages(self) -> None:
        """
        Start automatic status update messages from the controller.

        Once started, the controller will periodically send status update messages
        containing position, velocity, and status information. These can be captured
        by registering a callback for the update message type.
        """
        self.send_command(
            MsgId.HW_START_UPDATEMSGS,
            param1=0x00,
            param2=0x00,
            dest=0x11,  # Generic destination
            source=0x01
        )
        print("Started automatic status update messages")

    def stop_update_messages(self) -> None:
        """
        Stop automatic status update messages from the controller.

        This stops the periodic status updates that were started with
        start_update_messages().
        """
        self.send_command(
            MsgId.HW_STOP_UPDATEMSGS,
            param1=0x00,
            param2=0x00,
            dest=0x11,  # Generic destination
            source=0x01
        )
        print("Stopped automatic status update messages")

    def _ack_loop(self) -> None:
        """
        Background thread that sends periodic ACK messages.

        CRITICAL: The controller will stop responding after ~50 commands if ACK
        messages are not sent at least once per second.
        """
        while self._ack_running:
            try:
                self.send_command(
                    MsgId.MOT_ACK_USTATUSUPDATE,
                    param1=0x00,
                    param2=0x00,
                    dest=0x11,  # Generic destination
                    source=0x01
                )
                # Sleep for 1 second before next ACK
                time.sleep(1.0)
            except Exception as e:
                if self._ack_running:
                    print(f"ACK loop error: {e}")
                time.sleep(1.0)

    def start_status_ack(self) -> None:
        """
        Start the periodic status ACK thread.

        CRITICAL: This MUST be called to prevent the controller from stopping
        responses after ~50 commands. The ACK is sent every 1 second.
        """
        if self._ack_running:
            return  # Already running

        self._ack_running = True
        self._ack_thread = threading.Thread(target=self._ack_loop, daemon=True)
        self._ack_thread.start()
        print("Started periodic status ACK (every 1 second)")

    def stop_status_ack(self) -> None:
        """Stop the periodic status ACK thread."""
        if self._ack_running:
            self._ack_running = False
            if self._ack_thread:
                self._ack_thread.join(timeout=2.0)
                self._ack_thread = None
            print("Stopped periodic status ACK")

    def request_status_update(self, dest: int) -> None:
        """
        Request a status update from the specified axis.

        This sends MGMSG_MOT_REQ_USTATUSUPDATE which causes the controller to
        respond with MGMSG_MOT_GET_USTATUSUPDATE (0x0491).

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)

        Raises:
            ValueError: If dest is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x21 (X-axis) or 0x22 (Y-axis)")

        self.send_command(
            MsgId.MOT_REQ_USTATUSUPDATE,
            param1=0x01,
            param2=0x00,
            dest=dest,
            source=0x01
        )

    def ack_status_update(self) -> None:
        """
        Send a single status update ACK.

        Normally you should use start_status_ack() to run the ACK automatically.
        This method is for manual ACK control if needed.
        """
        self.send_command(
            MsgId.MOT_ACK_USTATUSUPDATE,
            param1=0x00,
            param2=0x00,
            dest=0x11,
            source=0x01
        )

    def set_channel_enable_state(self, dest: int, enabled: bool) -> None:
        """
        Set the channel enable state.

        Args:
            dest: Destination address (0x11 for X-axis, 0x22 for Y-axis)
            enabled: True to enable, False to disable

        Raises:
            ValueError: If dest is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x11 (X-axis) or 0x22 (Y-axis)")

        en_state = ChannelEnableState.ENABLED if enabled else ChannelEnableState.DISABLED

        self.send_command(
            MsgId.MOD_SET_CHANENABLESTATE,
            param1=0x02,  # Reserved byte
            param2=en_state,
            dest=dest,
            source=0x01
        )
    
    def get_channel_enable_state(self, dest: int, timeout: float = 5.0) -> Optional[bool]:
        """
        Get the channel enable state by sending REQ and waiting for GET response.
        
        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            timeout: Timeout in seconds
            
        Returns:
            True if enabled, False if disabled, None if timeout
            
        Raises:
            ValueError: If dest is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x21 (X-axis) or 0x22 (Y-axis)")
        
        # Create a unique waiter key combining message ID and expected source
        waiter_key = (MsgId.MOD_GET_CHANENABLESTATE, dest)
        event = threading.Event()
        responses = []
        
        with self._waiter_lock:
            self._waiters[waiter_key] = (event, responses)
        
        try:
            # Send REQ message to the axis, device will respond with GET
            self.send_command(
                MsgId.MOD_REQ_CHANENABLESTATE,
                param1=0x01,
                param2=0x00,
                dest=dest,  # Send to the axis
                source=0x01  # From us
            )
            
            # Wait for GET response from device
            if event.wait(timeout=timeout):
                response = responses[0] if responses else None
                if response is None:
                    return None
                en_state = response.param2
                return en_state == ChannelEnableState.ENABLED
            return None
        finally:
            with self._waiter_lock:
                if waiter_key in self._waiters:
                    del self._waiters[waiter_key]

    # Homing methods

    def home_axis(self, dest: int, timeout: float = 20.0) -> bool:
        """
        Home an axis by sending MOVE_HOME and waiting for MOVE_HOMED response.

        The controller will not respond until the axis is fully homed, which can
        take 10-15 seconds or more depending on the axis position. After homing,
        the position automatically resets to 0.

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            timeout: Timeout in seconds (default 20s to account for homing time)

        Returns:
            True if homing succeeded, False if timeout

        Raises:
            ValueError: If dest is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x21 (X-axis) or 0x22 (Y-axis)")

        # Create a unique waiter key combining message ID and expected source
        waiter_key = (MsgId.MOT_MOVE_HOMED, dest)
        event = threading.Event()
        responses = []

        with self._waiter_lock:
            self._waiters[waiter_key] = (event, responses)

        try:
            # Send MOVE_HOME message to the axis
            self.send_command(
                MsgId.MOT_MOVE_HOME,
                param1=0x01,  # Channel ID
                param2=0x00,
                dest=dest,  # Send to the axis
                source=0x01  # From us
            )

            print(f"Homing axis 0x{dest:02X}... (this may take 10-15 seconds)")

            # Wait for MOVE_HOMED response from device
            if event.wait(timeout=timeout):
                return True
            else:
                print(f"Timeout waiting for homing response from axis 0x{dest:02X}")
                return False
        finally:
            with self._waiter_lock:
                if waiter_key in self._waiters:
                    del self._waiters[waiter_key]

    def home_x_axis(self, timeout: float = 20.0) -> bool:
        """
        Home the X-axis. Position will reset to 0 after homing.

        Args:
            timeout: Timeout in seconds (default 20s)

        Returns:
            True if homing succeeded, False if timeout
        """
        return self.home_axis(self.DEST_X_AXIS, timeout=timeout)

    def home_y_axis(self, timeout: float = 20.0) -> bool:
        """
        Home the Y-axis. Position will reset to 0 after homing.

        Args:
            timeout: Timeout in seconds (default 20s)

        Returns:
            True if homing succeeded, False if timeout
        """
        return self.home_axis(self.DEST_Y_AXIS, timeout=timeout)

    # Velocity parameter methods

    def set_velocity_params(self, dest: int, min_velocity: float, acceleration: float,
                           max_velocity: float) -> None:
        """
        Set velocity parameters for an axis.

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            min_velocity: Minimum velocity in mm/s
            acceleration: Acceleration in mm/s²
            max_velocity: Maximum velocity in mm/s

        Raises:
            ValueError: If dest is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x21 (X-axis) or 0x22 (Y-axis)")

        # Convert from mm/s and mm/s² to encoder counts
        min_vel_counts = int(min_velocity * self.VELOCITY_SCALING)
        accel_counts = int(acceleration * self.ACCELERATION_SCALING)
        max_vel_counts = int(max_velocity * self.VELOCITY_SCALING)

        # Build the 14-byte data payload
        data = struct.pack('<HlLL',
            0x0001,           # Channel ID (bytes 6-7)
            min_vel_counts,   # Minimum velocity (bytes 8-11)
            accel_counts,     # Acceleration (bytes 12-15)
            max_vel_counts    # Maximum velocity (bytes 16-19)
        )

        self.send_command(
            MsgId.MOT_SET_VELPARAMS,
            data=data,
            dest=dest,
            source=0x01
        )

    def get_velocity_params(self, dest: int, timeout: float = 5.0) -> Optional[dict]:
        """
        Get velocity parameters for an axis.

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            timeout: Timeout in seconds

        Returns:
            Dictionary with keys: 'min_velocity', 'acceleration', 'max_velocity' (all in mm/s or mm/s²),
            or None if timeout

        Raises:
            ValueError: If dest is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x21 (X-axis) or 0x22 (Y-axis)")

        # Create a unique waiter key combining message ID and expected source
        waiter_key = (MsgId.MOT_GET_VELPARAMS, dest)
        event = threading.Event()
        responses = []

        with self._waiter_lock:
            self._waiters[waiter_key] = (event, responses)

        try:
            # Send REQ message to the axis
            self.send_command(
                MsgId.MOT_REQ_VELPARAMS,
                param1=0x01,
                param2=0x00,
                dest=dest,
                source=0x01
            )

            # Wait for GET response
            if event.wait(timeout=timeout):
                response = responses[0] if responses else None
                if response is None:
                    return None

                # Parse the 20-byte message
                if len(response.raw) >= 20:
                    channel_id = struct.unpack('<H', response.raw[6:8])[0]
                    min_vel_counts = struct.unpack('<l', response.raw[8:12])[0]
                    accel_counts = struct.unpack('<L', response.raw[12:16])[0]
                    max_vel_counts = struct.unpack('<L', response.raw[16:20])[0]

                    # Convert from encoder counts to mm/s and mm/s²
                    return {
                        'min_velocity': min_vel_counts / self.VELOCITY_SCALING,
                        'acceleration': accel_counts / self.ACCELERATION_SCALING,
                        'max_velocity': max_vel_counts / self.VELOCITY_SCALING
                    }
            return None
        finally:
            with self._waiter_lock:
                if waiter_key in self._waiters:
                    del self._waiters[waiter_key]

    def set_acceleration(self, dest: int, acceleration: float, timeout: float = 5.0) -> bool:
        """
        Set acceleration for an axis while preserving current velocity settings.

        This is a convenience method that retrieves current velocity parameters,
        then updates only the acceleration value.

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            acceleration: Acceleration in mm/s²
            timeout: Timeout for retrieving current parameters

        Returns:
            True if successful, False if unable to retrieve current parameters

        Raises:
            ValueError: If dest is invalid
        """
        # Get current velocity parameters
        current_params = self.get_velocity_params(dest, timeout=timeout)
        if current_params is None:
            return False

        # Set velocity parameters with new acceleration
        self.set_velocity_params(
            dest,
            min_velocity=current_params['min_velocity'],
            acceleration=acceleration,
            max_velocity=current_params['max_velocity']
        )
        return True

    def get_acceleration(self, dest: int, timeout: float = 5.0) -> Optional[float]:
        """
        Get acceleration for an axis.

        This is a convenience method that retrieves velocity parameters
        and returns only the acceleration value.

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            timeout: Timeout in seconds

        Returns:
            Acceleration in mm/s², or None if timeout

        Raises:
            ValueError: If dest is invalid
        """
        params = self.get_velocity_params(dest, timeout=timeout)
        if params is None:
            return None
        return params['acceleration']

    # Jog parameter methods

    def set_jog_params(self, dest: int, jog_mode: int, step_size: float,
                      min_velocity: float, acceleration: float, max_velocity: float,
                      stop_mode: int) -> None:
        """
        Set jog parameters for an axis.

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            jog_mode: Jog mode (JogMode.CONTINUOUS=1 or JogMode.SINGLE_STEP=2)
            step_size: Step size in mm (for single step mode)
            min_velocity: Minimum velocity in mm/s
            acceleration: Acceleration in mm/s²
            max_velocity: Maximum velocity in mm/s
            stop_mode: Stop mode (StopMode.IMMEDIATE=1 or StopMode.CONTROLLED=2)

        Raises:
            ValueError: If dest is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x21 (X-axis) or 0x22 (Y-axis)")

        # Convert from mm and mm/s and mm/s² to encoder counts
        step_size_counts = int(step_size * self.ENCODER_COUNTS_PER_MM)
        min_vel_counts = int(min_velocity * self.VELOCITY_SCALING)
        accel_counts = int(acceleration * self.ACCELERATION_SCALING)
        max_vel_counts = int(max_velocity * self.VELOCITY_SCALING)

        # Build the 22-byte data payload
        data = struct.pack('<HHLLLLH',
            0x0001,           # Channel ID (bytes 6-7)
            jog_mode,         # Jog mode (bytes 8-9)
            step_size_counts, # Step size (bytes 10-13)
            min_vel_counts,   # Minimum velocity (bytes 14-17)
            accel_counts,     # Acceleration (bytes 18-21)
            max_vel_counts,   # Maximum velocity (bytes 22-25)
            stop_mode         # Stop mode (bytes 26-27)
        )

        self.send_command(
            MsgId.MOT_SET_JOGPARAMS,
            data=data,
            dest=dest,
            source=0x01
        )

    def get_jog_params(self, dest: int, timeout: float = 5.0) -> Optional[dict]:
        """
        Get jog parameters for an axis.

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            timeout: Timeout in seconds

        Returns:
            Dictionary with keys: 'jog_mode', 'step_size' (mm), 'min_velocity' (mm/s),
            'acceleration' (mm/s²), 'max_velocity' (mm/s), 'stop_mode',
            or None if timeout

        Raises:
            ValueError: If dest is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x21 (X-axis) or 0x22 (Y-axis)")

        # Create a unique waiter key combining message ID and expected source
        waiter_key = (MsgId.MOT_GET_JOGPARAMS, dest)
        event = threading.Event()
        responses = []

        with self._waiter_lock:
            self._waiters[waiter_key] = (event, responses)

        try:
            # Send REQ message to the axis
            self.send_command(
                MsgId.MOT_REQ_JOGPARAMS,
                param1=0x01,
                param2=0x00,
                dest=dest,
                source=0x01
            )

            # Wait for GET response
            if event.wait(timeout=timeout):
                response = responses[0] if responses else None
                if response is None:
                    return None

                # Parse the 28-byte message
                if len(response.raw) >= 28:
                    channel_id = struct.unpack('<H', response.raw[6:8])[0]
                    jog_mode = struct.unpack('<H', response.raw[8:10])[0]
                    step_size_counts = struct.unpack('<L', response.raw[10:14])[0]
                    min_vel_counts = struct.unpack('<L', response.raw[14:18])[0]
                    accel_counts = struct.unpack('<L', response.raw[18:22])[0]
                    max_vel_counts = struct.unpack('<L', response.raw[22:26])[0]
                    stop_mode = struct.unpack('<H', response.raw[26:28])[0]

                    # Convert from encoder counts to mm and mm/s and mm/s²
                    return {
                        'jog_mode': jog_mode,
                        'step_size': step_size_counts / self.ENCODER_COUNTS_PER_MM,
                        'step_size_counts': step_size_counts,
                        'min_velocity': min_vel_counts / self.VELOCITY_SCALING,
                        'acceleration': accel_counts / self.ACCELERATION_SCALING,
                        'max_velocity': max_vel_counts / self.VELOCITY_SCALING,
                        'stop_mode': stop_mode
                    }
            return None
        finally:
            with self._waiter_lock:
                if waiter_key in self._waiters:
                    del self._waiters[waiter_key]

    # Move relative parameter methods

    def set_move_rel_params(self, dest: int, relative_distance: float) -> None:
        """
        Set relative move distance for an axis.

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            relative_distance: Relative distance to move in mm (positive or negative)

        Raises:
            ValueError: If dest is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x21 (X-axis) or 0x22 (Y-axis)")

        # Convert from mm to encoder counts (signed)
        relative_distance_counts = int(relative_distance * self.ENCODER_COUNTS_PER_MM)

        # Build the 6-byte data payload
        data = struct.pack('<Hi',
            0x0001,                    # Channel ID (bytes 6-7)
            relative_distance_counts   # Relative distance (bytes 8-11)
        )

        self.send_command(
            MsgId.MOT_SET_MOVERELPARAMS,
            data=data,
            dest=dest,
            source=0x01
        )

    def get_move_rel_params(self, dest: int, timeout: float = 5.0) -> Optional[dict]:
        """
        Get relative move parameters for an axis.

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            timeout: Timeout in seconds

        Returns:
            Dictionary with keys: 'relative_distance' (mm), 'relative_distance_counts',
            or None if timeout

        Raises:
            ValueError: If dest is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x21 (X-axis) or 0x22 (Y-axis)")

        # Create a unique waiter key combining message ID and expected source
        waiter_key = (MsgId.MOT_GET_MOVERELPARAMS, dest)
        event = threading.Event()
        responses = []

        with self._waiter_lock:
            self._waiters[waiter_key] = (event, responses)

        try:
            # Send REQ message to the axis
            self.send_command(
                MsgId.MOT_REQ_MOVERELPARAMS,
                param1=0x01,
                param2=0x00,
                dest=dest,
                source=0x01
            )

            # Wait for GET response
            if event.wait(timeout=timeout):
                response = responses[0] if responses else None
                if response is None:
                    return None

                # Parse the 12-byte message
                if len(response.raw) >= 12:
                    channel_id = struct.unpack('<H', response.raw[6:8])[0]
                    relative_distance_counts = struct.unpack('<i', response.raw[8:12])[0]

                    # Convert from encoder counts to mm
                    return {
                        'relative_distance': relative_distance_counts / self.ENCODER_COUNTS_PER_MM,
                        'relative_distance_counts': relative_distance_counts
                    }
            return None
        finally:
            with self._waiter_lock:
                if waiter_key in self._waiters:
                    del self._waiters[waiter_key]

    # Move absolute parameter methods

    def set_move_abs_params(self, dest: int, absolute_position: float) -> None:
        """
        Set absolute move position for an axis.

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            absolute_position: Absolute position to move to in mm

        Raises:
            ValueError: If dest is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x21 (X-axis) or 0x22 (Y-axis)")

        # Convert from mm to encoder counts (signed)
        absolute_position_counts = int(absolute_position * self.ENCODER_COUNTS_PER_MM)

        # Build the 6-byte data payload
        data = struct.pack('<Hi',
            0x0001,                    # Channel ID (bytes 6-7)
            absolute_position_counts   # Absolute position (bytes 8-11)
        )

        self.send_command(
            MsgId.MOT_SET_MOVEABSPARAMS,
            data=data,
            dest=dest,
            source=0x01
        )

    def get_move_abs_params(self, dest: int, timeout: float = 5.0) -> Optional[dict]:
        """
        Get absolute move parameters for an axis.

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            timeout: Timeout in seconds

        Returns:
            Dictionary with keys: 'absolute_position' (mm), 'absolute_position_counts',
            or None if timeout

        Raises:
            ValueError: If dest is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x21 (X-axis) or 0x22 (Y-axis)")

        # Create a unique waiter key combining message ID and expected source
        waiter_key = (MsgId.MOT_GET_MOVEABSPARAMS, dest)
        event = threading.Event()
        responses = []

        with self._waiter_lock:
            self._waiters[waiter_key] = (event, responses)

        try:
            # Send REQ message to the axis
            self.send_command(
                MsgId.MOT_REQ_MOVEABSPARAMS,
                param1=0x01,
                param2=0x00,
                dest=dest,
                source=0x01
            )

            # Wait for GET response
            if event.wait(timeout=timeout):
                response = responses[0] if responses else None
                if response is None:
                    return None

                # Parse the 12-byte message
                if len(response.raw) >= 12:
                    channel_id = struct.unpack('<H', response.raw[6:8])[0]
                    absolute_position_counts = struct.unpack('<i', response.raw[8:12])[0]

                    # Convert from encoder counts to mm
                    return {
                        'absolute_position': absolute_position_counts / self.ENCODER_COUNTS_PER_MM,
                        'absolute_position_counts': absolute_position_counts
                    }
            return None
        finally:
            with self._waiter_lock:
                if waiter_key in self._waiters:
                    del self._waiters[waiter_key]

    # Move execution methods

    def move_relative(self, dest: int, timeout: float = 30.0) -> Optional[dict]:
        """
        Execute a relative move using the previously set relative move parameters.

        The controller will not respond until the move is completed. This command
        uses the relative distance that was previously set with set_move_rel_params().

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            timeout: Timeout in seconds (default 30s to account for move time)

        Returns:
            Dictionary with move completion data: 'position' (mm), 'position_counts',
            'velocity', 'motor_current', 'status_bits', or None if timeout

        Raises:
            ValueError: If dest is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x21 (X-axis) or 0x22 (Y-axis)")

        # Create a unique waiter key combining message ID and expected source
        waiter_key = (MsgId.MOT_MOVE_COMPLETED, dest)
        event = threading.Event()
        responses = []

        with self._waiter_lock:
            self._waiters[waiter_key] = (event, responses)

        try:
            # Send MOVE_RELATIVE command (header only)
            self.send_command(
                MsgId.MOT_MOVE_RELATIVE,
                param1=0x01,
                param2=0x00,
                dest=dest,
                source=0x01
            )

            print(f"Executing relative move on axis 0x{dest:02X}... (this may take some time)")

            # Wait for MOVE_COMPLETED response from device
            if event.wait(timeout=timeout):
                response = responses[0] if responses else None
                if response is None:
                    return None
                # The move completion data should already be stored in _move_completed by _parse_buffer
                with self._move_completed_lock:
                    return self._move_completed.get(dest)
            else:
                print(f"Timeout waiting for move completion from axis 0x{dest:02X}")
                return None
        finally:
            with self._waiter_lock:
                if waiter_key in self._waiters:
                    del self._waiters[waiter_key]

    def move_absolute(self, dest: int, timeout: float = 30.0) -> Optional[dict]:
        """
        Execute an absolute move using the previously set absolute move parameters.

        The controller will not respond until the move is completed. This command
        uses the absolute position that was previously set with set_move_abs_params().

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            timeout: Timeout in seconds (default 30s to account for move time)

        Returns:
            Dictionary with move completion data: 'position' (mm), 'position_counts',
            'velocity', 'motor_current', 'status_bits', or None if timeout

        Raises:
            ValueError: If dest is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x21 (X-axis) or 0x22 (Y-axis)")

        # Create a unique waiter key combining message ID and expected source
        waiter_key = (MsgId.MOT_MOVE_COMPLETED, dest)
        event = threading.Event()
        responses = []

        with self._waiter_lock:
            self._waiters[waiter_key] = (event, responses)

        try:
            # Send MOVE_ABSOLUTE command (header only)
            self.send_command(
                MsgId.MOT_MOVE_ABSOLUTE,
                param1=0x01,
                param2=0x00,
                dest=dest,
                source=0x01
            )

            print(f"Executing absolute move on axis 0x{dest:02X}... (this may take some time)")

            # Wait for MOVE_COMPLETED response from device
            if event.wait(timeout=timeout):
                response = responses[0] if responses else None
                if response is None:
                    return None
                # The move completion data should already be stored in _move_completed by _parse_buffer
                with self._move_completed_lock:
                    return self._move_completed.get(dest)
            else:
                print(f"Timeout waiting for move completion from axis 0x{dest:02X}")
                return None
        finally:
            with self._waiter_lock:
                if waiter_key in self._waiters:
                    del self._waiters[waiter_key]

    def stop_move(self, dest: int, stop_mode: int = StopMode.CONTROLLED,
                  wait_for_stopped: bool = True, timeout: float = 5.0) -> Optional[dict]:
        """
        Stop a moving axis immediately or with controlled deceleration.

        Args:
            dest: Destination address (0x21 for X-axis, 0x22 for Y-axis)
            stop_mode: StopMode.IMMEDIATE (1) for instant stop,
                      StopMode.CONTROLLED (2) for controlled deceleration (default)
            wait_for_stopped: If True, wait for MOVE_STOPPED message (default True)
            timeout: Timeout in seconds when waiting for MOVE_STOPPED (default 5s)

        Returns:
            If wait_for_stopped=True: Dictionary with stopped status data, or None if timeout
            If wait_for_stopped=False: None immediately after sending command

        Raises:
            ValueError: If dest or stop_mode is invalid
        """
        if dest not in (self.DEST_X_AXIS, self.DEST_Y_AXIS):
            raise ValueError(f"Invalid destination: 0x{dest:02X}. Must be 0x21 (X-axis) or 0x22 (Y-axis)")

        if stop_mode not in (StopMode.IMMEDIATE, StopMode.CONTROLLED):
            raise ValueError(f"Invalid stop_mode: {stop_mode}. Must be StopMode.IMMEDIATE (1) or StopMode.CONTROLLED (2)")

        if wait_for_stopped:
            # Create a unique waiter key combining message ID and expected source
            waiter_key = (MsgId.MOT_MOVE_STOPPED, dest)
            event = threading.Event()
            responses = []

            with self._waiter_lock:
                self._waiters[waiter_key] = (event, responses)

            try:
                # Send MOVE_STOP command (header only)
                self.send_command(
                    MsgId.MOT_MOVE_STOP,
                    param1=0x01,  # Channel ID (always 1)
                    param2=stop_mode,  # Stop mode
                    dest=dest,
                    source=0x01
                )

                print(f"Stopping axis 0x{dest:02X} with {'immediate' if stop_mode == StopMode.IMMEDIATE else 'controlled'} stop...")

                # Wait for MOVE_STOPPED response from device
                if event.wait(timeout=timeout):
                    response = responses[0] if responses else None
                    if response is None:
                        return None
                    # The move stopped data should already be stored in _move_stopped by _parse_buffer
                    with self._move_stopped_lock:
                        return self._move_stopped.get(dest)
                else:
                    print(f"Timeout waiting for stop confirmation from axis 0x{dest:02X}")
                    return None
            finally:
                with self._waiter_lock:
                    if waiter_key in self._waiters:
                        del self._waiters[waiter_key]
        else:
            # Just send the command without waiting
            self.send_command(
                MsgId.MOT_MOVE_STOP,
                param1=0x01,  # Channel ID (always 1)
                param2=stop_mode,  # Stop mode
                dest=dest,
                source=0x01
            )
            return None

    def stop_x_axis(self, stop_mode: int = StopMode.CONTROLLED,
                    wait_for_stopped: bool = True, timeout: float = 5.0) -> Optional[dict]:
        """
        Stop the X-axis.

        Args:
            stop_mode: StopMode.IMMEDIATE (1) or StopMode.CONTROLLED (2, default)
            wait_for_stopped: If True, wait for MOVE_STOPPED message (default True)
            timeout: Timeout in seconds when waiting (default 5s)

        Returns:
            Dictionary with stopped status data if wait_for_stopped=True, None otherwise
        """
        return self.stop_move(self.DEST_X_AXIS, stop_mode, wait_for_stopped, timeout)

    def stop_y_axis(self, stop_mode: int = StopMode.CONTROLLED,
                    wait_for_stopped: bool = True, timeout: float = 5.0) -> Optional[dict]:
        """
        Stop the Y-axis.

        Args:
            stop_mode: StopMode.IMMEDIATE (1) or StopMode.CONTROLLED (2, default)
            wait_for_stopped: If True, wait for MOVE_STOPPED message (default True)
            timeout: Timeout in seconds when waiting (default 5s)

        Returns:
            Dictionary with stopped status data if wait_for_stopped=True, None otherwise
        """
        return self.stop_move(self.DEST_Y_AXIS, stop_mode, wait_for_stopped, timeout)

    def stop_all_axes(self, stop_mode: int = StopMode.CONTROLLED,
                      wait_for_stopped: bool = True, timeout: float = 5.0) -> dict:
        """
        Stop all axes simultaneously.

        Args:
            stop_mode: StopMode.IMMEDIATE (1) or StopMode.CONTROLLED (2, default)
            wait_for_stopped: If True, wait for MOVE_STOPPED messages (default True)
            timeout: Timeout in seconds when waiting (default 5s)

        Returns:
            Dictionary with keys 'x' and 'y' containing stopped status data or None
        """
        if wait_for_stopped:
            # Use threading to stop both axes simultaneously
            x_result = [None]
            y_result = [None]

            def stop_x():
                x_result[0] = self.stop_move(self.DEST_X_AXIS, stop_mode, True, timeout)

            def stop_y():
                y_result[0] = self.stop_move(self.DEST_Y_AXIS, stop_mode, True, timeout)

            x_thread = threading.Thread(target=stop_x)
            y_thread = threading.Thread(target=stop_y)

            x_thread.start()
            y_thread.start()

            x_thread.join()
            y_thread.join()

            return {'x': x_result[0], 'y': y_result[0]}
        else:
            # Just send commands without waiting
            self.stop_move(self.DEST_X_AXIS, stop_mode, False)
            self.stop_move(self.DEST_Y_AXIS, stop_mode, False)
            return {'x': None, 'y': None}

    # Status bit decoding methods

    @staticmethod
    def decode_status_bits(status_bits: int) -> MotorStatusBits:
        """
        Decode status bits into a MotorStatusBits IntFlag.

        Args:
            status_bits: Raw 32-bit status bits value

        Returns:
            MotorStatusBits IntFlag with all active bits set
        """
        return MotorStatusBits(status_bits)

    @staticmethod
    def get_status_description(status_bits: int) -> str:
        """
        Get a human-readable description of all active status bits.

        Args:
            status_bits: Raw 32-bit status bits value

        Returns:
            Multi-line string describing all active status bits
        """
        status = MotorStatusBits(status_bits)
        descriptions = []

        # Error conditions (highest priority)
        if MotorStatusBits.COMMUTATIONERROR in status:
            descriptions.append("ERROR: Motor commutation error - power cycle required")
        if MotorStatusBits.OVERTEMP in status:
            descriptions.append("ERROR: Overtemperature detected")
        if MotorStatusBits.BUSVOLTFAULT in status:
            descriptions.append("ERROR: Supply voltage too low")
        if MotorStatusBits.OVERLOAD in status:
            descriptions.append("ERROR: Motor overload/overcurrent")
        if MotorStatusBits.ENCODERFAULT in status:
            descriptions.append("ERROR: Encoder fault")
        if MotorStatusBits.OVERCURRENT in status:
            descriptions.append("ERROR: Continuous current limit exceeded")
        if MotorStatusBits.ERROR in status:
            descriptions.append("ERROR: Other error condition")

        # Limit switches
        if MotorStatusBits.CWHARDLIMIT in status:
            descriptions.append("WARNING: Clockwise hard limit triggered")
        if MotorStatusBits.CCWHARDLIMIT in status:
            descriptions.append("WARNING: Counter-clockwise hard limit triggered")
        if MotorStatusBits.CWSOFTLIMIT in status:
            descriptions.append("WARNING: Clockwise software limit triggered")
        if MotorStatusBits.CCWSOFTLIMIT in status:
            descriptions.append("WARNING: Counter-clockwise software limit triggered")
        if MotorStatusBits.POSITIONERROR in status:
            descriptions.append("WARNING: Position error - outside tracking window")

        # Motion state
        if MotorStatusBits.HOMING in status:
            descriptions.append("STATUS: Motor is homing")
        elif MotorStatusBits.INMOTIONCW in status:
            descriptions.append("STATUS: Moving clockwise")
        elif MotorStatusBits.INMOTIONCCW in status:
            descriptions.append("STATUS: Moving counter-clockwise")
        elif MotorStatusBits.ACTIVE in status:
            descriptions.append("STATUS: Executing motion command")
        elif MotorStatusBits.SETTLED in status:
            descriptions.append("STATUS: Settled at target position")

        # Operational state
        if MotorStatusBits.INITIALIZING in status:
            descriptions.append("STATUS: Performing phase initialization")
        if MotorStatusBits.HOMED in status:
            descriptions.append("STATUS: Homed (position count valid)")
        if MotorStatusBits.TRACKING in status:
            descriptions.append("STATUS: Position within tracking window")
        if MotorStatusBits.CONNECTED in status:
            descriptions.append("STATUS: Motor recognized by controller")
        if MotorStatusBits.ENABLED in status:
            descriptions.append("STATUS: Motor output enabled")
        if MotorStatusBits.POWEROK in status:
            descriptions.append("STATUS: Power supply OK")

        if not descriptions:
            descriptions.append("STATUS: No status bits set")

        return "\n".join(descriptions)

    @staticmethod
    def has_errors(status_bits: int) -> bool:
        """
        Check if any error conditions are present in status bits.

        Args:
            status_bits: Raw 32-bit status bits value

        Returns:
            True if any error bits are set, False otherwise
        """
        status = MotorStatusBits(status_bits)
        error_flags = (
            MotorStatusBits.OVERTEMP |
            MotorStatusBits.BUSVOLTFAULT |
            MotorStatusBits.COMMUTATIONERROR |
            MotorStatusBits.OVERLOAD |
            MotorStatusBits.ENCODERFAULT |
            MotorStatusBits.OVERCURRENT |
            MotorStatusBits.ERROR
        )
        return bool(status & error_flags)

    @staticmethod
    def is_in_motion(status_bits: int) -> bool:
        """
        Check if motor is currently in motion.

        Args:
            status_bits: Raw 32-bit status bits value

        Returns:
            True if motor is moving, False otherwise
        """
        status = MotorStatusBits(status_bits)
        motion_flags = (
            MotorStatusBits.INMOTIONCW |
            MotorStatusBits.INMOTIONCCW |
            MotorStatusBits.HOMING |
            MotorStatusBits.ACTIVE
        )
        return bool(status & motion_flags)

    @staticmethod
    def is_settled(status_bits: int) -> bool:
        """
        Check if motor is settled at target position.

        Args:
            status_bits: Raw 32-bit status bits value

        Returns:
            True if motor is settled, False otherwise
        """
        status = MotorStatusBits(status_bits)
        return MotorStatusBits.SETTLED in status

    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False


if __name__ == '__main__':
    # Example: monitor status updates and control channel enable state
    def on_status_update(msg: AptMessage):
        print(f"Status update: msg_id=0x{msg.msg_id:04X}, data={msg.data.hex() if msg.data else 'none'}")

    def on_move_stopped(msg: AptMessage):
        """Callback for unexpected motor stop events."""
        print(f"\n!!! MOVE STOPPED EVENT DETECTED !!!")
        print(f"    Source: 0x{msg.source:02X} ({'X-axis' if msg.source == 0x21 else 'Y-axis' if msg.source == 0x22 else 'Unknown'})")
        # The status will already be parsed and stored in _move_stopped
        # Note: You can access mc.move_stopped_x or mc.move_stopped_y properties
        # after this callback to get the full stopped status including status bits

    with MotionController() as mc:
        # Register callbacks
        mc.register_callback(MsgId.MOT_GET_DCSTATUSUPDATE, on_status_update)
        mc.register_callback(MsgId.MOT_MOVE_STOPPED, on_move_stopped)

        # Print hardware info
        print("Hardware Info:")
        print(f"  Serial Number: {mc.get_serial_number()}")
        print(f"  Model: {mc.get_model()}")
        print(f"  Firmware: {mc.get_firmware_version()}")
        print(f"  Channels: {mc.get_num_channels()}")

        print("\n=== Homing Test ===")
        print("Homing X-axis...")
        try:
            print("  Enabling X-axis channel...")
            mc.set_channel_enable_state(mc.DEST_X_AXIS, enabled=True)
            time.sleep(0.5)

            if mc.home_x_axis(timeout=20.0):
                print(f"  X-axis homed successfully")
                print(f"  Position: {mc.position_x} mm (encoder count: {mc.encoder_count_x})")
            else:
                print(f"  X-axis homing timed out")
        except Exception as e:
            print(f"  Error homing X-axis: {e}")

        print("\nHoming Y-axis...")
        try:
            print("  Enabling Y-axis channel...")
            mc.set_channel_enable_state(mc.DEST_Y_AXIS, enabled=True)
            time.sleep(0.5)

            if mc.home_y_axis(timeout=20.0):
                print(f"  Y-axis homed successfully")
                print(f"  Position: {mc.position_y} mm (encoder count: {mc.encoder_count_y})")
            else:
                print(f"  Y-axis homing timed out")
        except Exception as e:
            print(f"  Error homing Y-axis: {e}")

        print("\n=== Jog Parameters Test ===")
        print("Getting current jog parameters for X-axis...")
        try:
            jog_params = mc.get_jog_params(mc.DEST_X_AXIS, timeout=2.0)
            if jog_params:
                print(f"  Jog mode: {jog_params['jog_mode']} ({'Continuous' if jog_params['jog_mode'] == JogMode.CONTINUOUS else 'Single Step'})")
                print(f"  Step size: {jog_params['step_size']:.3f} mm ({jog_params['step_size_counts']} counts)")
                print(f"  Min velocity: {jog_params['min_velocity']:.3f} mm/s")
                print(f"  Acceleration: {jog_params['acceleration']:.3f} mm/s²")
                print(f"  Max velocity: {jog_params['max_velocity']:.3f} mm/s")
                print(f"  Stop mode: {jog_params['stop_mode']} ({'Immediate' if jog_params['stop_mode'] == StopMode.IMMEDIATE else 'Controlled'})")
            else:
                print(f"  Timeout getting jog parameters")
        except Exception as e:
            print(f"  Error getting jog parameters: {e}")

        print("\nUsing jog parameter properties:")
        print(f"  X-axis jog params: {mc.jog_params_x}")

        print("\nSetting new jog parameters for X-axis...")
        try:
            # Set: single step mode, 1mm step, min_vel=0.5 mm/s, accel=50 mm/s², max_vel=10 mm/s, controlled stop
            mc.set_jog_params(
                mc.DEST_X_AXIS,
                jog_mode=JogMode.SINGLE_STEP,
                step_size=1.0,
                min_velocity=0.5,
                acceleration=50.0,
                max_velocity=10.0,
                stop_mode=StopMode.CONTROLLED
            )
            time.sleep(0.2)

            # Read back the parameters
            jog_params = mc.get_jog_params(mc.DEST_X_AXIS, timeout=2.0)
            if jog_params:
                print(f"  New jog mode: {jog_params['jog_mode']} ({'Continuous' if jog_params['jog_mode'] == JogMode.CONTINUOUS else 'Single Step'})")
                print(f"  New step size: {jog_params['step_size']:.3f} mm")
                print(f"  New min velocity: {jog_params['min_velocity']:.3f} mm/s")
                print(f"  New acceleration: {jog_params['acceleration']:.3f} mm/s²")
                print(f"  New max velocity: {jog_params['max_velocity']:.3f} mm/s")
                print(f"  New stop mode: {jog_params['stop_mode']} ({'Immediate' if jog_params['stop_mode'] == StopMode.IMMEDIATE else 'Controlled'})")
            else:
                print(f"  Timeout reading back jog parameters")

            # Show cached properties updated
            print(f"\n  Cached properties after update:")
            print(f"    X jog params: {mc.jog_params_x}")
        except Exception as e:
            print(f"  Error setting jog parameters: {e}")

        print("\n=== Move Relative Parameters Test ===")
        print("Getting current move relative parameters for X-axis...")
        try:
            move_rel_params = mc.get_move_rel_params(mc.DEST_X_AXIS, timeout=2.0)
            if move_rel_params:
                print(f"  Relative distance: {move_rel_params['relative_distance']:.3f} mm ({move_rel_params['relative_distance_counts']} counts)")
            else:
                print(f"  Timeout getting move relative parameters")
        except Exception as e:
            print(f"  Error getting move relative parameters: {e}")

        print("\nUsing move relative parameter properties:")
        print(f"  X-axis move rel params: {mc.move_rel_params_x}")
        print(f"  X relative distance: {mc.relative_distance_x} mm")

        print("\nSetting new move relative parameters for X-axis...")
        try:
            # Set: relative distance of 5mm
            mc.set_move_rel_params(mc.DEST_X_AXIS, relative_distance=5.0)
            time.sleep(0.2)

            # Read back the parameters
            move_rel_params = mc.get_move_rel_params(mc.DEST_X_AXIS, timeout=2.0)
            if move_rel_params:
                print(f"  New relative distance: {move_rel_params['relative_distance']:.3f} mm ({move_rel_params['relative_distance_counts']} counts)")
            else:
                print(f"  Timeout reading back move relative parameters")

            # Show cached properties updated
            print(f"\n  Cached properties after update:")
            print(f"    X relative distance: {mc.relative_distance_x} mm")
        except Exception as e:
            print(f"  Error setting move relative parameters: {e}")

        print("\nTesting negative relative move...")
        try:
            # Set: relative distance of -2.5mm
            mc.set_move_rel_params(mc.DEST_X_AXIS, relative_distance=-2.5)
            time.sleep(0.2)

            # Read back the parameters
            move_rel_params = mc.get_move_rel_params(mc.DEST_X_AXIS, timeout=2.0)
            if move_rel_params:
                print(f"  New relative distance: {move_rel_params['relative_distance']:.3f} mm ({move_rel_params['relative_distance_counts']} counts)")
                print(f"  Cached X relative distance: {mc.relative_distance_x} mm")
            else:
                print(f"  Timeout reading back move relative parameters")
        except Exception as e:
            print(f"  Error setting negative move relative parameters: {e}")

        print("\n=== Move Absolute Parameters Test ===")
        print("Getting current move absolute parameters for X-axis...")
        try:
            move_abs_params = mc.get_move_abs_params(mc.DEST_X_AXIS, timeout=2.0)
            if move_abs_params:
                print(f"  Absolute position: {move_abs_params['absolute_position']:.3f} mm ({move_abs_params['absolute_position_counts']} counts)")
            else:
                print(f"  Timeout getting move absolute parameters")
        except Exception as e:
            print(f"  Error getting move absolute parameters: {e}")

        print("\nUsing move absolute parameter properties:")
        print(f"  X-axis move abs params: {mc.move_abs_params_x}")
        print(f"  X absolute position: {mc.absolute_position_x} mm")

        print("\nSetting new move absolute parameters for X-axis...")
        try:
            # Set: absolute position of 10mm
            mc.set_move_abs_params(mc.DEST_X_AXIS, absolute_position=10.0)
            time.sleep(0.2)

            # Read back the parameters
            move_abs_params = mc.get_move_abs_params(mc.DEST_X_AXIS, timeout=2.0)
            if move_abs_params:
                print(f"  New absolute position: {move_abs_params['absolute_position']:.3f} mm ({move_abs_params['absolute_position_counts']} counts)")
            else:
                print(f"  Timeout reading back move absolute parameters")

            # Show cached properties updated
            print(f"\n  Cached properties after update:")
            print(f"    X absolute position: {mc.absolute_position_x} mm")
        except Exception as e:
            print(f"  Error setting move absolute parameters: {e}")

        print("\nTesting different absolute position...")
        try:
            # Set: absolute position of 45mm (near the home position)
            mc.set_move_abs_params(mc.DEST_X_AXIS, absolute_position=45.0)
            time.sleep(0.2)

            # Read back the parameters
            move_abs_params = mc.get_move_abs_params(mc.DEST_X_AXIS, timeout=2.0)
            if move_abs_params:
                print(f"  New absolute position: {move_abs_params['absolute_position']:.3f} mm ({move_abs_params['absolute_position_counts']} counts)")
                print(f"  Cached X absolute position: {mc.absolute_position_x} mm")
            else:
                print(f"  Timeout reading back move absolute parameters")
        except Exception as e:
            print(f"  Error setting absolute position: {e}")

        print("\n=== Move Relative Execution Test ===")
        print("Setting up relative move of 2mm for X-axis...")
        try:
            # First, set the relative move parameters
            mc.set_move_rel_params(mc.DEST_X_AXIS, relative_distance=2.0)
            time.sleep(0.2)

            # Verify parameters were set
            rel_params = mc.get_move_rel_params(mc.DEST_X_AXIS, timeout=2.0)
            if rel_params:
                print(f"  Relative move parameters set: {rel_params['relative_distance']:.3f} mm")
            else:
                print(f"  Failed to verify relative move parameters")

            # Execute the relative move
            print("\n  Executing relative move...")
            move_result = mc.move_relative(mc.DEST_X_AXIS, timeout=30.0)

            if move_result:
                print(f"  Move completed successfully!")
                print(f"    Final position: {move_result['position']:.3f} mm ({move_result['position_counts']} counts)")
                print(f"    Final velocity: {move_result['velocity']} encoder units")
                print(f"    Motor current: {move_result['motor_current']}")
                print(f"    Status bits: 0x{move_result['status_bits']:08X}")

                # Decode and display status bits
                print(f"\n  Status bit analysis:")
                status_desc = MotionController.get_status_description(move_result['status_bits'])
                for line in status_desc.split('\n'):
                    print(f"    {line}")

                print(f"\n  Status bit checks:")
                print(f"    Has errors: {MotionController.has_errors(move_result['status_bits'])}")
                print(f"    Is in motion: {MotionController.is_in_motion(move_result['status_bits'])}")
                print(f"    Is settled: {MotionController.is_settled(move_result['status_bits'])}")

                # Show cached property
                print(f"\n  Cached move completed status:")
                print(f"    X: {mc.move_completed_x}")

                # Demonstrate status bit properties
                print(f"\n  Status bit properties (automatically updated):")
                print(f"    Raw status bits: 0x{mc.status_bits_x:08X}" if mc.status_bits_x else "    Raw status bits: Not available")
                print(f"    Is enabled: {mc.is_enabled_x}")
                print(f"    Is homed: {mc.is_homed_x}")
                print(f"    Is in motion: {mc.is_in_motion_x}")
                print(f"    Is settled: {mc.is_settled_x}")
                print(f"    Is tracking: {mc.is_tracking_x}")
                print(f"    Is connected: {mc.is_connected_x}")
                print(f"    Power OK: {mc.power_ok_x}")
                print(f"    Is active: {mc.is_active_x}")
                print(f"    Has errors: {mc.has_errors_x}")
                print(f"    At CW limit: {mc.at_cw_limit_x}")
                print(f"    At CCW limit: {mc.at_ccw_limit_x}")
            else:
                print(f"  Move timed out or failed")
        except Exception as e:
            print(f"  Error executing relative move: {e}")

        print("\n=== Status Bit Properties Summary ===")
        print("X-axis status:")
        if mc.status_bits_x:
            print(f"  Raw bits: 0x{mc.status_bits_x:08X}")
            print(f"  Enabled: {mc.is_enabled_x}, Homed: {mc.is_homed_x}, Connected: {mc.is_connected_x}")
            print(f"  In motion: {mc.is_in_motion_x}, Settled: {mc.is_settled_x}, Active: {mc.is_active_x}")
            print(f"  Has errors: {mc.has_errors_x}, Power OK: {mc.power_ok_x}")
        else:
            print("  No status available yet")

        print("\nY-axis status:")
        if mc.status_bits_y:
            print(f"  Raw bits: 0x{mc.status_bits_y:08X}")
            print(f"  Enabled: {mc.is_enabled_y}, Homed: {mc.is_homed_y}, Connected: {mc.is_connected_y}")
            print(f"  In motion: {mc.is_in_motion_y}, Settled: {mc.is_settled_y}, Active: {mc.is_active_y}")
            print(f"  Has errors: {mc.has_errors_y}, Power OK: {mc.power_ok_y}")
        else:
            print("  No status available yet")

        print("\n=== Test Complete ===")
        print("Waiting 1 second before exit to allow final messages...")
        time.sleep(1.0)
