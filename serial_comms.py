'''
    SRAS Serial Communication Tools
    Thomas Ales | Feb 2026
    Version 1
'''
from threading import Thread
from queue import Queue
import serial
import struct
import time

class SerialSnooper(Thread):

    def __init__(self, _port, _spd):
        super().__init__()
        self.serial_connection = None
        self.serial_port = _port
        self.serial_speed = _spd
        self.am_listening = False
        self.rx_msg_queue = Queue()

    def run(self):
        '''
            run() - Starts up the serial listener, checks if the port was
                    opened successfully, and if so begins
                    listening for APT messages.
        '''
        self.serial_connection = serial.Serial(port=self.serial_port,
                                               baudrate=self.serial_speed,
                                               rtscts=True, timeout=0.05)
        if self.serial_connection.is_open is True:
            # Send disconnect to stop any ongoing auto-updates from
            # a previous session
            for addr in [0x11, 0x21, 0x22]:
                disconnect_msg = struct.pack('<HBBBB', 0x0002,
                                             0x00, 0x00, addr, 0x01)
                self.serial_connection.write(disconnect_msg)

            # Wait for controller to process, then flush everything
            time.sleep(0.2)
            self.serial_connection.reset_output_buffer()
            self.serial_connection.reset_input_buffer()

            # Discard any remaining data that arrived
            time.sleep(0.1)
            if self.serial_connection.in_waiting > 0:
                self.serial_connection.read(self.serial_connection.in_waiting)

            self.am_listening = True
            _rxbuf = bytearray()
            while self.am_listening is True:
                try:
                    # Read whatever is available (or wait up to timeout)
                    _chunk = self.serial_connection.read(
                        max(1, self.serial_connection.in_waiting))
                    if _chunk:
                        _rxbuf.extend(_chunk)

                    # Process complete messages from the buffer
                    while len(_rxbuf) >= 6:
                        # Check if this is a long message (bit 7 of byte 4)
                        if _rxbuf[4] & 0x80:
                            _msglen = struct.unpack("<H", _rxbuf[2:4])[0]
                            total = 6 + _msglen
                            if len(_rxbuf) < total:
                                break  # need more bytes
                            _packet = bytes(_rxbuf[:total])
                            del _rxbuf[:total]
                        else:
                            _packet = bytes(_rxbuf[:6])
                            del _rxbuf[:6]

                        self.rx_msg_queue.put_nowait(_packet)

                except (serial.SerialException, TypeError, OSError):
                    break
        return

    def stop(self):
        '''
            stop() - Signals the listener loop to stop. Call join() after
            this to wait for the thread to exit, then call close() to
            release the serial port.
        '''
        self.am_listening = False

    def close(self):
        '''
            close() - Closes the serial port. Only call after the
            thread has been joined.
        '''
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()

