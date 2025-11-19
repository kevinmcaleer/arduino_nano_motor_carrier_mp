"""
Battery monitoring class for Motor Carrier.
Port of Battery.h/cpp from Arduino library.
"""

import struct
from .common import Command


class Battery:
    """
    Battery voltage monitoring via I2C.
    Provides raw, converted, and filtered ADC readings.

    Note: getConverted() and getFiltered() return values calculated by the
    Motor Carrier firmware. For accurate voltage, use get_voltage() which
    calculates from the raw ADC value.
    """

    # Conversion factor from Arduino examples: voltage = raw / 236.0
    VOLTAGE_CONVERSION_FACTOR = 236.0

    def __init__(self, i2c_comm):
        """
        Initialize Battery monitor.

        Args:
            i2c_comm: I2CComm object for communication
        """
        self.comm = i2c_comm

    def get_raw(self):
        """
        Get raw ADC value from battery voltage sensor.

        Returns:
            Raw ADC value (signed 32-bit integer) or None on failure
        """
        data = self.comm.read_register(Command.GET_RAW_ADC_BATTERY, 0x00, 4)
        if data:
            return struct.unpack('<i', data)[0]
        return None

    def get_converted(self):
        """
        Get converted battery voltage value.

        Returns:
            Converted voltage value (signed 32-bit integer) or None on failure
        """
        data = self.comm.read_register(Command.GET_CONVERTED_ADC_BATTERY, 0x00, 4)
        if data:
            return struct.unpack('<i', data)[0]
        return None

    def get_filtered(self):
        """
        Get filtered battery voltage value (smoothed reading).

        Returns:
            Filtered voltage value (signed 32-bit integer) or None on failure
        """
        data = self.comm.read_register(Command.GET_FILTERED_ADC_BATTERY, 0x00, 4)
        if data:
            return struct.unpack('<i', data)[0]
        return None

    def get_voltage(self):
        """
        Calculate battery voltage from raw ADC reading.

        This uses the conversion formula from the Arduino examples:
        voltage = raw_adc / 236.0

        Returns:
            Battery voltage in volts, or None on failure
        """
        raw = self.get_raw()
        if raw is not None:
            return raw / self.VOLTAGE_CONVERSION_FACTOR
        return None

    def is_low_battery(self, threshold=11.0):
        """
        Check if battery voltage is below threshold.

        Args:
            threshold: Voltage threshold in volts (default 11.0V)

        Returns:
            True if battery is low, False if OK, None if reading failed
        """
        voltage = self.get_voltage()
        if voltage is not None:
            return voltage < threshold
        return None

    def get_status(self):
        """
        Get battery status information.

        Returns:
            Dictionary with battery information or None on failure
        """
        raw = self.get_raw()
        if raw is None:
            return None

        voltage = raw / self.VOLTAGE_CONVERSION_FACTOR

        # Determine status
        if voltage < 6.0:
            status = "NO_BATTERY"
            message = "No battery connected or critically low"
        elif voltage < 11.0:
            status = "LOW"
            message = "Battery voltage low - recharge recommended"
        elif voltage < 11.5:
            status = "MEDIUM"
            message = "Battery voltage medium"
        else:
            status = "GOOD"
            message = "Battery voltage good"

        return {
            'raw': raw,
            'voltage': voltage,
            'status': status,
            'message': message,
            'converted': self.get_converted(),
            'filtered': self.get_filtered()
        }
