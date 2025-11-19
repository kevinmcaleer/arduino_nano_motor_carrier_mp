"""
Motor Carrier library for MicroPython (ESP32).
Port of Arduino Motor Carrier library.

This library provides control for:
- 4 DC Motors (M1, M2 via I2C; M3, M4 via GPIO)
- 4 Servo Motors (via I2C)
- 2 Encoders (via I2C)
- 2 PID Controllers (I2C-based or local)
- Battery monitoring

Usage:
    from motor_carrier import MotorCarrier

    mc = MotorCarrier()
    mc.begin()

    # Use motors
    mc.M1.set_duty(50)  # 50% forward
    mc.servo1.set_angle(90)  # 90 degrees

    # Read encoder
    count = mc.encoder1.get_raw_count()
"""

from .common import ControlMode, TargetType, Pins
from .i2c_comm import I2CComm
from .controller import MotorController
from .dc_motor import DCMotorI2C, DCMotorGPIO
from .servo_motor import ServoMotor
from .encoder import Encoder
from .battery import Battery
from .pid import PIDI2C, PIDLocal


class MotorCarrier:
    """
    Main Motor Carrier interface.
    Provides Arduino-like API with global objects.
    """

    def __init__(self, i2c=None, scl=12, sda=11,
                 m3_pins=(2, 3), m4_pins=(5, 4)):
        """
        Initialize Motor Carrier.

        Args:
            i2c: Existing I2C object (optional)
            scl: I2C SCL pin (default 12)
            sda: I2C SDA pin (default 11)
            m3_pins: Tuple of (pin_a, pin_b) for M3 motor GPIO control (default: 2, 3 - Nano 33 IoT compatible)
            m4_pins: Tuple of (pin_a, pin_b) for M4 motor GPIO control (default: 5, 4 - Nano 33 IoT compatible)
        """
        # I2C communication
        self.comm = I2CComm(i2c, scl, sda)

        # Motor Controller
        self.controller = MotorController(self.comm)

        # DC Motors (I2C-based: M1, M2)
        self.M1 = DCMotorI2C(self.comm)
        self.M2 = DCMotorI2C(self.comm)

        # DC Motors (GPIO-based: M3, M4)
        self.M3 = DCMotorGPIO(m3_pins[0], m3_pins[1])
        self.M4 = DCMotorGPIO(m4_pins[0], m4_pins[1])

        # Servo Motors
        self.servo1 = ServoMotor(self.comm)
        self.servo2 = ServoMotor(self.comm)
        self.servo3 = ServoMotor(self.comm)
        self.servo4 = ServoMotor(self.comm)

        # Encoders
        self.encoder1 = Encoder(self.comm)
        self.encoder2 = Encoder(self.comm)

        # Battery Monitor
        self.battery = Battery(self.comm)

        # PID Controllers (I2C-based)
        self.pid1 = PIDI2C(self.comm)
        self.pid2 = PIDI2C(self.comm)

        # PID Controllers (Local - created on demand)
        self._pid1_local = None
        self._pid2_local = None

    def begin(self, enable_charging=False):
        """
        Initialize Motor Carrier.

        Args:
            enable_charging: Enable battery charging (for Nano 33 IoT compatibility)

        Returns:
            0 on success, 1 on failure
        """
        return self.controller.begin(enable_charging)

    def create_local_pid(self, encoder_num, motor_num, period_ms_velo=10, period_ms_pos=10):
        """
        Create a local PID controller (d21 equivalent).

        Args:
            encoder_num: Encoder number (1 or 2)
            motor_num: Motor number (3 or 4 for GPIO motors)
            period_ms_velo: Velocity loop period in ms
            period_ms_pos: Position loop period in ms

        Returns:
            PIDLocal object
        """
        encoder = self.encoder1 if encoder_num == 1 else self.encoder2
        motor = self.M3 if motor_num == 3 else self.M4

        pid = PIDLocal(encoder, motor, period_ms_velo, period_ms_pos)

        # Store reference
        if encoder_num == 1:
            self._pid1_local = pid
        else:
            self._pid2_local = pid

        return pid


# Export commonly used classes and constants
__all__ = [
    'MotorCarrier',
    'MotorController',
    'DCMotorI2C',
    'DCMotorGPIO',
    'ServoMotor',
    'Encoder',
    'Battery',
    'PIDI2C',
    'PIDLocal',
    'ControlMode',
    'TargetType',
    'Pins',
]
