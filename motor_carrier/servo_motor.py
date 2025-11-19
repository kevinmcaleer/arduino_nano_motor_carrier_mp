"""
Servo Motor control class for Motor Carrier.
Port of ServoMotor.h/cpp from Arduino library.
"""

from .common import Command


class ServoMotor:
    """
    Servo Motor control via I2C.
    Supports 4 servo motors controlled via SAMD11 coprocessor.
    """

    _next_instance = 0

    def __init__(self, i2c_comm):
        """
        Initialize Servo Motor.

        Args:
            i2c_comm: I2CComm object for communication
        """
        self.comm = i2c_comm
        self.instance = ServoMotor._next_instance
        ServoMotor._next_instance += 1

    def set_angle(self, angle):
        """
        Set servo position.

        Args:
            angle: Servo angle from 0 to 180 degrees
        """
        # Clamp angle to valid range
        angle = max(0, min(180, angle))

        # Map angle (0-180) to duty cycle (7-28)
        # This mapping matches the Arduino library
        duty = int(self._map(angle, 0, 180, 7, 28))

        self.comm.write_register(Command.SET_PWM_DUTY_CYCLE_SERVO, self.instance, duty)

    def detach(self):
        """
        Detach servo (disable PWM signal).
        """
        # Send -1 to detach servo
        self.comm.write_register(Command.SET_PWM_DUTY_CYCLE_SERVO, self.instance, -1)

    def set_frequency(self, frequency):
        """
        Set PWM frequency for servo control.

        Args:
            frequency: PWM frequency in Hz
        """
        self.comm.write_register(Command.SET_PWM_FREQUENCY_SERVO, self.instance, frequency)

    @staticmethod
    def _map(value, in_min, in_max, out_min, out_max):
        """
        Map a value from one range to another (Arduino map function).

        Args:
            value: Input value
            in_min: Input range minimum
            in_max: Input range maximum
            out_min: Output range minimum
            out_max: Output range maximum

        Returns:
            Mapped value in output range
        """
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
