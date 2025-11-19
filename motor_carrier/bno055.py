"""
BNO055 9-axis IMU driver for Motor Carrier.
Supports accelerometer, gyroscope, magnetometer, and sensor fusion.

Based on Bosch BNO055 datasheet and register map.
"""

import struct
import time
from micropython import const

# BNO055 I2C Address
BNO055_ADDRESS = const(0x28)

# BNO055 Registers
BNO055_CHIP_ID = const(0x00)
BNO055_PAGE_ID = const(0x07)
BNO055_ACCEL_DATA = const(0x08)
BNO055_MAG_DATA = const(0x0E)
BNO055_GYRO_DATA = const(0x14)
BNO055_EULER_DATA = const(0x1A)
BNO055_QUAT_DATA = const(0x20)
BNO055_LINEAR_ACCEL_DATA = const(0x28)
BNO055_GRAVITY_DATA = const(0x2E)
BNO055_TEMP = const(0x34)
BNO055_CALIB_STAT = const(0x35)
BNO055_SYS_STATUS = const(0x39)
BNO055_SYS_ERR = const(0x3A)
BNO055_UNIT_SEL = const(0x3B)
BNO055_OPR_MODE = const(0x3D)
BNO055_PWR_MODE = const(0x3E)
BNO055_SYS_TRIGGER = const(0x3F)

# Operation Modes
OPERATION_MODE_CONFIG = const(0x00)
OPERATION_MODE_ACCONLY = const(0x01)
OPERATION_MODE_MAGONLY = const(0x02)
OPERATION_MODE_GYRONLY = const(0x03)
OPERATION_MODE_ACCMAG = const(0x04)
OPERATION_MODE_ACCGYRO = const(0x05)
OPERATION_MODE_MAGGYRO = const(0x06)
OPERATION_MODE_AMG = const(0x07)
OPERATION_MODE_IMUPLUS = const(0x08)
OPERATION_MODE_COMPASS = const(0x09)
OPERATION_MODE_M4G = const(0x0A)
OPERATION_MODE_NDOF_FMC_OFF = const(0x0B)
OPERATION_MODE_NDOF = const(0x0C)

# Power Modes
POWER_MODE_NORMAL = const(0x00)
POWER_MODE_LOWPOWER = const(0x01)
POWER_MODE_SUSPEND = const(0x02)


class BNO055:
    """
    BNO055 9-axis IMU (Accelerometer, Gyroscope, Magnetometer).

    Features:
    - 3-axis accelerometer
    - 3-axis gyroscope
    - 3-axis magnetometer
    - Sensor fusion (orientation as Euler angles or quaternions)
    - Built-in calibration
    """

    def __init__(self, i2c, address=BNO055_ADDRESS):
        """
        Initialize BNO055 IMU.

        Args:
            i2c: I2C object
            address: I2C address (default 0x28)
        """
        self.i2c = i2c
        self.address = address
        self._mode = OPERATION_MODE_CONFIG

    def begin(self):
        """
        Initialize the BNO055 sensor.

        Returns:
            True on success, False on failure
        """
        # Check chip ID
        chip_id = self._read_register(BNO055_CHIP_ID)
        if chip_id != 0xA0:
            print(f"BNO055 chip ID error: expected 0xA0, got 0x{chip_id:02x}")
            return False

        # Reset to default
        self._write_register(BNO055_SYS_TRIGGER, 0x20)
        time.sleep(0.65)  # Wait for reset

        # Set to config mode
        self.set_mode(OPERATION_MODE_CONFIG)
        time.sleep(0.02)

        # Set normal power mode
        self._write_register(BNO055_PWR_MODE, POWER_MODE_NORMAL)
        time.sleep(0.01)

        # Use internal oscillator
        self._write_register(BNO055_SYS_TRIGGER, 0x00)
        time.sleep(0.01)

        # Set units: Android orientation mode, Celsius, Degrees, m/s²
        self._write_register(BNO055_UNIT_SEL, 0x00)
        time.sleep(0.01)

        # Set to NDOF mode (9-axis fusion)
        self.set_mode(OPERATION_MODE_NDOF)

        return True

    def set_mode(self, mode):
        """
        Set operation mode.

        Args:
            mode: Operation mode constant (e.g., OPERATION_MODE_NDOF)
        """
        self._write_register(BNO055_OPR_MODE, mode)
        self._mode = mode
        time.sleep(0.03)  # Mode switching delay

    def get_calibration_status(self):
        """
        Get calibration status for all sensors.

        Returns:
            Dictionary with calibration status (0-3, where 3 is fully calibrated)
            {'system': 0-3, 'gyro': 0-3, 'accel': 0-3, 'mag': 0-3}
        """
        cal_status = self._read_register(BNO055_CALIB_STAT)
        return {
            'system': (cal_status >> 6) & 0x03,
            'gyro': (cal_status >> 4) & 0x03,
            'accel': (cal_status >> 2) & 0x03,
            'mag': cal_status & 0x03
        }

    def is_calibrated(self):
        """
        Check if sensor is fully calibrated.

        Returns:
            True if all sensors are calibrated (status = 3)
        """
        cal = self.get_calibration_status()
        return all(v == 3 for v in cal.values())

    def get_temperature(self):
        """
        Get temperature in degrees Celsius.

        Returns:
            Temperature in °C
        """
        return self._read_register(BNO055_TEMP)

    def get_euler(self):
        """
        Get orientation as Euler angles (heading, roll, pitch).

        Returns:
            Dictionary {'heading': yaw, 'roll': roll, 'pitch': pitch} in degrees
            or None on failure
        """
        data = self._read_bytes(BNO055_EULER_DATA, 6)
        if data:
            # Data is in 1/16th degree units (LSB = 1/16 degree)
            h = struct.unpack('<h', data[0:2])[0] / 16.0  # Heading (yaw)
            r = struct.unpack('<h', data[2:4])[0] / 16.0  # Roll
            p = struct.unpack('<h', data[4:6])[0] / 16.0  # Pitch
            return {'heading': h, 'roll': r, 'pitch': p}
        return None

    def get_quaternion(self):
        """
        Get orientation as quaternion.

        Returns:
            Dictionary {'w': w, 'x': x, 'y': y, 'z': z} or None on failure
        """
        data = self._read_bytes(BNO055_QUAT_DATA, 8)
        if data:
            # Data is in 1/16384 scale
            w = struct.unpack('<h', data[0:2])[0] / 16384.0
            x = struct.unpack('<h', data[2:4])[0] / 16384.0
            y = struct.unpack('<h', data[4:6])[0] / 16384.0
            z = struct.unpack('<h', data[6:8])[0] / 16384.0
            return {'w': w, 'x': x, 'y': y, 'z': z}
        return None

    def get_accelerometer(self):
        """
        Get accelerometer data.

        Returns:
            Dictionary {'x': x, 'y': y, 'z': z} in m/s² or None on failure
        """
        data = self._read_bytes(BNO055_ACCEL_DATA, 6)
        if data:
            # Data is in 1 m/s² = 100 LSB
            x = struct.unpack('<h', data[0:2])[0] / 100.0
            y = struct.unpack('<h', data[2:4])[0] / 100.0
            z = struct.unpack('<h', data[4:6])[0] / 100.0
            return {'x': x, 'y': y, 'z': z}
        return None

    def get_gyroscope(self):
        """
        Get gyroscope data.

        Returns:
            Dictionary {'x': x, 'y': y, 'z': z} in deg/s or None on failure
        """
        data = self._read_bytes(BNO055_GYRO_DATA, 6)
        if data:
            # Data is in 1 deg/s = 16 LSB
            x = struct.unpack('<h', data[0:2])[0] / 16.0
            y = struct.unpack('<h', data[2:4])[0] / 16.0
            z = struct.unpack('<h', data[4:6])[0] / 16.0
            return {'x': x, 'y': y, 'z': z}
        return None

    def get_magnetometer(self):
        """
        Get magnetometer data.

        Returns:
            Dictionary {'x': x, 'y': y, 'z': z} in µT or None on failure
        """
        data = self._read_bytes(BNO055_MAG_DATA, 6)
        if data:
            # Data is in 1 µT = 16 LSB
            x = struct.unpack('<h', data[0:2])[0] / 16.0
            y = struct.unpack('<h', data[2:4])[0] / 16.0
            z = struct.unpack('<h', data[4:6])[0] / 16.0
            return {'x': x, 'y': y, 'z': z}
        return None

    def get_linear_acceleration(self):
        """
        Get linear acceleration (with gravity removed).

        Returns:
            Dictionary {'x': x, 'y': y, 'z': z} in m/s² or None on failure
        """
        data = self._read_bytes(BNO055_LINEAR_ACCEL_DATA, 6)
        if data:
            # Data is in 1 m/s² = 100 LSB
            x = struct.unpack('<h', data[0:2])[0] / 100.0
            y = struct.unpack('<h', data[2:4])[0] / 100.0
            z = struct.unpack('<h', data[4:6])[0] / 100.0
            return {'x': x, 'y': y, 'z': z}
        return None

    def get_gravity(self):
        """
        Get gravity vector.

        Returns:
            Dictionary {'x': x, 'y': y, 'z': z} in m/s² or None on failure
        """
        data = self._read_bytes(BNO055_GRAVITY_DATA, 6)
        if data:
            # Data is in 1 m/s² = 100 LSB
            x = struct.unpack('<h', data[0:2])[0] / 100.0
            y = struct.unpack('<h', data[2:4])[0] / 100.0
            z = struct.unpack('<h', data[4:6])[0] / 100.0
            return {'x': x, 'y': y, 'z': z}
        return None

    def get_system_status(self):
        """
        Get system status.

        Returns:
            Dictionary with status info or None on failure
        """
        sys_status = self._read_register(BNO055_SYS_STATUS)
        sys_err = self._read_register(BNO055_SYS_ERR)

        status_msgs = {
            0: "System Idle",
            1: "System Error",
            2: "Initializing Peripherals",
            3: "System Initialization",
            4: "Executing Self-Test",
            5: "Sensor Fusion Running",
            6: "System Running (No Fusion)"
        }

        error_msgs = {
            0: "No Error",
            1: "Peripheral Init Error",
            2: "System Init Error",
            3: "Self Test Failed",
            4: "Register Map Value Out of Range",
            5: "Register Map Address Out of Range",
            6: "Register Map Write Error",
            7: "Low Power Mode Not Available",
            8: "Accelerometer Power Mode Not Available",
            9: "Fusion Config Error",
            10: "Sensor Config Error"
        }

        return {
            'status': sys_status,
            'status_msg': status_msgs.get(sys_status, "Unknown"),
            'error': sys_err,
            'error_msg': error_msgs.get(sys_err, "Unknown Error")
        }

    def _read_register(self, register):
        """Read a single byte from a register."""
        try:
            self.i2c.writeto(self.address, bytes([register]))
            return self.i2c.readfrom(self.address, 1)[0]
        except OSError:
            return None

    def _read_bytes(self, register, num_bytes):
        """Read multiple bytes from a register."""
        try:
            self.i2c.writeto(self.address, bytes([register]))
            return self.i2c.readfrom(self.address, num_bytes)
        except OSError:
            return None

    def _write_register(self, register, value):
        """Write a single byte to a register."""
        try:
            self.i2c.writeto(self.address, bytes([register, value]))
            return True
        except OSError:
            return False
