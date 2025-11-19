"""
Encoder class for Motor Carrier.
Port of Encoder.h/cpp from Arduino library.
"""

import struct
from .common import Command


class Encoder:
    """
    Encoder interface for reading position and velocity.
    Supports 2 encoders via I2C communication with SAMD11 coprocessor.
    """

    _next_instance = 0

    def __init__(self, i2c_comm):
        """
        Initialize Encoder.

        Args:
            i2c_comm: I2CComm object for communication
        """
        self.comm = i2c_comm
        self.instance = Encoder._next_instance
        Encoder._next_instance += 1

    def get_raw_count(self):
        """
        Get raw encoder count.

        Returns:
            Encoder count (signed 32-bit integer) or None on failure
        """
        data = self.comm.read_register(Command.GET_RAW_COUNT_ENCODER, self.instance, 4)
        if data:
            return struct.unpack('<i', data)[0]
        return None

    def get_count_per_second(self):
        """
        Get encoder velocity in counts per second.

        Returns:
            Velocity in counts/sec (signed 32-bit integer) or None on failure
        """
        data = self.comm.read_register(Command.GET_COUNT_PER_SECOND_ENCODER, self.instance, 4)
        if data:
            return struct.unpack('<i', data)[0]
        return None

    def get_overflow_underflow(self):
        """
        Get overflow/underflow status.

        Returns:
            Status value (16-bit) or None on failure
            Bit 0-7: Underflow count
            Bit 8-15: Overflow count
        """
        data = self.comm.read_register(Command.GET_OVERFLOW_UNDERFLOW_STATUS_ENCODER, self.instance, 2)
        if data and len(data) >= 2:
            # Arduino code: ret[0] << 8 | ret[1]
            return (data[0] << 8) | data[1]
        return None

    def reset_counter(self, value=0):
        """
        Reset encoder counter to a specific value.

        Args:
            value: Value to reset counter to (default 0)
        """
        self.comm.write_register(Command.RESET_COUNT_ENCODER, self.instance, value)

    def set_irq_on_count(self, value):
        """
        Set interrupt to trigger when encoder reaches a specific count.

        Args:
            value: Count value to trigger interrupt
        """
        self.comm.write_register(Command.SET_INTERRUPT_ON_COUNT_ENCODER, self.instance, value)

    def set_irq_on_velocity(self, value, margin=2):
        """
        Set interrupt to trigger when encoder velocity reaches a specific value.

        Args:
            value: Velocity value to trigger interrupt (counts/sec)
            margin: Margin around target velocity (default 2)
        """
        # Combine margin and value: (margin << 24) | value
        combined = (margin << 24) | (value & 0xFFFFFF)
        self.comm.write_register(Command.SET_INTERRUPT_ON_VELOCITY_ENCODER, self.instance, combined)
