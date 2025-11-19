"""
I2C communication utilities for Motor Carrier.
Port of ArduinoMotorCarrier.cpp I2C functions.
"""

from machine import I2C, Pin
import struct
import time
from .common import MOTOR_CARRIER_ADDRESS, Command

class I2CComm:
    """Low-level I2C communication with Motor Carrier."""

    def __init__(self, i2c=None, scl=12, sda=11, freq=100000):
        """
        Initialize I2C communication.

        Args:
            i2c: Existing I2C object (optional)
            scl: SCL pin number (default 22 for ESP32)
            sda: SDA pin number (default 21 for ESP32)
            freq: I2C frequency in Hz (default 100kHz)
        """
        if i2c is None:
            self.i2c = I2C(0, scl=Pin(scl), sda=Pin(sda), freq=freq)
        else:
            self.i2c = i2c
        self.address = MOTOR_CARRIER_ADDRESS

    def write_register(self, command, target, data=None):
        """
        Write command to Motor Carrier.

        Args:
            command: Command byte (from Command class)
            target: Target device ID (motor number, servo number, etc.)
            data: Optional data bytes (int, list, or bytes)

        Returns:
            True on success, False on failure
        """
        try:
            # Build command buffer
            buffer = bytearray([command, target])

            # Append data if provided
            if data is not None:
                if isinstance(data, int):
                    # Single byte or convert to bytes
                    if -128 <= data <= 127:
                        buffer.append(data & 0xFF)
                    else:
                        # Multi-byte integer (little-endian, handle signed)
                        value = data
                        if value < 0:
                            value = (1 << 32) + value  # Convert to unsigned
                        buffer.extend(value.to_bytes(4, 'little'))
                elif isinstance(data, (list, bytes, bytearray)):
                    buffer.extend(data)

            # Write to I2C
            self.i2c.writeto(self.address, buffer)
            return True
        except OSError as e:
            print(f"I2C write error: {e}")
            return False

    def read_register(self, command, target, num_bytes=4):
        """
        Read data from Motor Carrier.

        Args:
            command: Command byte (from Command class)
            target: Target device ID
            num_bytes: Number of bytes to read (default 4)

        Returns:
            Bytes object with response data, or None on failure
        """
        try:
            # Write command
            buffer = bytearray([command, target])
            self.i2c.writeto(self.address, buffer)

            # Small delay for Motor Carrier to prepare response
            time.sleep_ms(5)

            # Read response (first byte is IRQ status, rest is data)
            response = self.i2c.readfrom(self.address, num_bytes + 1)

            # Return data (skip IRQ byte)
            return response[1:]
        except OSError as e:
            print(f"I2C read error: {e}")
            return None

    def read_version(self):
        """
        Read firmware version string.

        Returns:
            Version string or None on failure
        """
        try:
            buffer = bytearray([Command.GET_VERSION, 0x00])
            self.i2c.writeto(self.address, buffer)

            # Small delay for Motor Carrier to prepare response
            time.sleep_ms(10)

            # Read version string (up to 16 bytes)
            response = self.i2c.readfrom(self.address, 16)

            # Convert to string (skip IRQ byte)
            version_bytes = response[1:]

            # Find terminator (null byte 0x00 or padding 0xFF)
            null_idx = version_bytes.find(b'\x00')
            if null_idx >= 0:
                version_bytes = version_bytes[:null_idx]
            else:
                # No null terminator, look for 0xFF padding
                for i, b in enumerate(version_bytes):
                    if b == 0xFF:
                        version_bytes = version_bytes[:i]
                        break

            # Try to decode
            try:
                return version_bytes.decode('ascii')
            except (UnicodeError, ValueError):
                # If decode fails, extract printable characters only
                result = ''
                for b in version_bytes:
                    if 32 <= b <= 126:  # Printable ASCII
                        result += chr(b)
                return result if result else None
        except OSError as e:
            print(f"I2C version read error: {e}")
            return None

    def ping(self):
        """
        Send ping command to keep communication alive.

        Returns:
            True on success, False on failure
        """
        return self.write_register(Command.PING, 0x00)

    def reset(self):
        """
        Reset the Motor Carrier controller.

        Returns:
            True on success, False on failure
        """
        return self.write_register(Command.RESET, 0x00)

    def get_temperature(self):
        """
        Read internal temperature of Motor Carrier.

        Returns:
            Temperature value or None on failure
        """
        data = self.read_register(Command.GET_INTERNAL_TEMP, 0x00, 4)
        if data:
            return struct.unpack('<i', data)[0]
        return None

    def get_free_ram(self):
        """
        Read free RAM on Motor Carrier coprocessor.

        Returns:
            Free RAM in bytes or None on failure
        """
        data = self.read_register(Command.GET_FREE_RAM, 0x00, 4)
        if data:
            return struct.unpack('<I', data)[0]
        return None

    def clear_irq(self):
        """
        Clear interrupt status.

        Returns:
            True on success, False on failure
        """
        return self.write_register(Command.CLEAR_IRQ, 0x00)

    def get_irq_status(self):
        """
        Get interrupt status.

        Returns:
            IRQ status byte or None on failure
        """
        try:
            buffer = bytearray([Command.CLEAR_IRQ, 0x00])
            self.i2c.writeto(self.address, buffer)
            response = self.i2c.readfrom(self.address, 1)
            return response[0]
        except OSError as e:
            print(f"I2C IRQ status error: {e}")
            return None
