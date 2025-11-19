"""
DC Motor control classes for Motor Carrier.
Port of DCMotor.h/cpp from Arduino library.

Two implementations:
- mc.DCMotor: I2C-based control (M1, M2) via SAMD11 coprocessor
- d21.DCMotor: Direct GPIO/PWM control (M3, M4)
"""

from machine import Pin, PWM
from .common import Command


class DCMotorI2C:
    """
    DC Motor with I2C control (mc namespace equivalent).
    Used for M1 and M2 motors controlled via SAMD11 coprocessor.
    """

    _next_instance = 0

    def __init__(self, i2c_comm):
        """
        Initialize DC Motor with I2C control.

        Args:
            i2c_comm: I2CComm object for communication
        """
        self.comm = i2c_comm
        self.instance = DCMotorI2C._next_instance
        DCMotorI2C._next_instance += 1

    def set_duty(self, duty):
        """
        Set motor speed and direction.

        Args:
            duty: Motor duty cycle from -100 to +100
                  Positive = forward, Negative = reverse, 0 = stop
        """
        # Clamp duty to valid range
        duty = max(-100, min(100, duty))
        self.comm.write_register(Command.SET_PWM_DUTY_CYCLE_DC_MOTOR, self.instance, duty)

    def set_frequency(self, frequency):
        """
        Set PWM frequency for motor control.

        Args:
            frequency: PWM frequency in Hz
        """
        self.comm.write_register(Command.SET_PWM_FREQUENCY_DC_MOTOR, self.instance, frequency)


class DCMotorGPIO:
    """
    DC Motor with direct GPIO/PWM control (d21 namespace equivalent).
    Used for M3 and M4 motors controlled directly via GPIO pins.
    """

    _next_instance = 0

    def __init__(self, pin_a, pin_b):
        """
        Initialize DC Motor with GPIO control.

        Args:
            pin_a: First motor control pin (IN1 or IN3)
            pin_b: Second motor control pin (IN2 or IN4)
        """
        self.instance = DCMotorGPIO._next_instance
        DCMotorGPIO._next_instance += 1

        self.pin_a = pin_a
        self.pin_b = pin_b
        self.duty = 0

        # Initialize PWM on both pins
        self.pwm_a = PWM(Pin(pin_a), freq=1000, duty=0)
        self.pwm_b = PWM(Pin(pin_b), freq=1000, duty=0)

    def set_duty(self, duty):
        """
        Set motor speed and direction using GPIO PWM.

        Args:
            duty: Motor duty cycle from -100 to +100
                  Positive = forward, Negative = reverse, 0 = stop
        """
        # Clamp duty to valid range
        duty = max(-100, min(100, duty))
        self.duty = duty

        if duty == 0:
            # Stop motor
            self.pwm_a.duty(0)
            self.pwm_b.duty(0)
        elif duty > 0:
            # Forward direction
            self.pwm_a.duty(0)
            # Map 0-100 to 0-1023 (ESP32 PWM range)
            pwm_value = int((duty / 100.0) * 1023)
            self.pwm_b.duty(pwm_value)
        else:
            # Reverse direction
            self.pwm_b.duty(0)
            # Map 0-100 to 0-1023
            pwm_value = int((-duty / 100.0) * 1023)
            self.pwm_a.duty(pwm_value)

    def set_frequency(self, frequency):
        """
        Set PWM frequency for motor control.

        Args:
            frequency: PWM frequency in Hz
        """
        self.pwm_a.freq(frequency)
        self.pwm_b.freq(frequency)

    def deinit(self):
        """Deinitialize PWM pins."""
        self.pwm_a.deinit()
        self.pwm_b.deinit()
