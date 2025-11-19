"""
MotorController class for Motor Carrier.
Port of MotorController.h/cpp from Arduino library.
"""

import time
from .i2c_comm import I2CComm
from .common import PMIC_ADDRESS, Command

# PMIC Register addresses for battery charging
PMIC_REG00 = 0x00
PMIC_REG01 = 0x01
PMIC_REG02 = 0x02
PMIC_REG04 = 0x04
PMIC_REG05 = 0x05


class MotorController:
    """
    Main controller class for Motor Carrier.
    Handles initialization, firmware version, temperature monitoring, etc.
    """

    def __init__(self, i2c_comm=None):
        """
        Initialize MotorController.

        Args:
            i2c_comm: I2CComm object (optional, will create if not provided)
        """
        self.comm = i2c_comm if i2c_comm else I2CComm()
        self.irq_status = 0

    def begin(self, enable_charging=False):
        """
        Initialize the Motor Carrier.

        Args:
            enable_charging: Enable battery charging (for Nano 33 IoT compatibility)

        Returns:
            0 on success, 1 on failure
        """
        # Enable battery charging if requested
        if enable_charging:
            print("Enabling battery charging...")
            self.enable_battery_charging()

        # Check firmware version
        version = self.get_fw_version()
        if version is None or len(version) == 0:
            print("Failed to communicate with Motor Carrier")
            return 1

        print(f"Motor Carrier firmware version: {version}")
        return 0

    def get_fw_version(self):
        """
        Get firmware version string.

        Returns:
            Version string or None on failure
        """
        return self.comm.read_version()

    def reboot(self):
        """
        Reboot the Motor Carrier controller.
        Waits 500ms after reboot for controller to initialize.
        """
        self.comm.reset()
        time.sleep(0.5)

    def ping(self):
        """
        Send ping command to keep communication alive.

        Returns:
            True on success, False on failure
        """
        return self.comm.ping()

    def get_temperature(self):
        """
        Read internal temperature of Motor Carrier.

        Returns:
            Temperature in degrees Celsius or None if not available
        """
        temp = self.comm.get_temperature()
        if temp is not None and temp != -1:
            # Convert from millidegrees to degrees
            return temp / 1000.0
        return None  # Temperature sensor not available or not supported

    def get_irq_status(self):
        """
        Get interrupt status.

        Returns:
            IRQ status byte
        """
        status = self.comm.get_irq_status()
        if status is not None:
            self.irq_status = status
        return self.irq_status

    def get_free_ram(self):
        """
        Get free RAM on Motor Carrier coprocessor.

        Returns:
            Free RAM in bytes or None on failure
        """
        return self.comm.get_free_ram()

    def enable_battery_charging(self):
        """
        Enable battery charging via PMIC.
        This is specific to Nano 33 IoT boards.

        Charging parameters:
        - Min system voltage: 3.88V
        - Max input current: 2.0A
        - Charge current: 512mA
        - Charge voltage limit: 4.128V
        """
        try:
            i2c = self.comm.i2c

            # REG00: min sys voltage 3.88V + max input current 2.0A
            i2c.writeto(PMIC_ADDRESS, bytes([PMIC_REG00, 0x06]))

            # REG01: Charge Battery + Minimum System Voltage 3.5V
            i2c.writeto(PMIC_ADDRESS, bytes([PMIC_REG01, 0x1B]))

            # REG02: Charge current 512mA
            i2c.writeto(PMIC_ADDRESS, bytes([PMIC_REG02, 0x00]))

            # REG04: Charge Voltage Limit 4.128V
            i2c.writeto(PMIC_ADDRESS, bytes([PMIC_REG04, 0x9E]))

            # REG05: Enable Battery Charge termination + disable watchdog
            i2c.writeto(PMIC_ADDRESS, bytes([PMIC_REG05, 0x8A]))

            return True
        except OSError as e:
            print(f"PMIC configuration error: {e}")
            return False
