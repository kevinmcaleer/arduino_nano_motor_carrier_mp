"""
PID control classes for Motor Carrier.
Port of PID.h/cpp from Arduino library.

Two implementations:
- PIDI2C: I2C-based PID (mc namespace equivalent) - runs on SAMD11
- PIDLocal: Local PID (d21 namespace equivalent) - runs on main board
"""

import struct
import time
from .common import Command, ControlMode, TargetType, fixed_to_bytes, bytes_to_fixed, fixed_to_float


class PIDI2C:
    """
    PID controller with I2C control (mc namespace equivalent).
    PID computation runs on SAMD11 coprocessor.
    """

    _next_instance = 0

    def __init__(self, i2c_comm):
        """
        Initialize PID controller with I2C control.

        Args:
            i2c_comm: I2CComm object for communication
        """
        self.comm = i2c_comm
        self.instance = PIDI2C._next_instance
        PIDI2C._next_instance += 1

    def set_gains(self, kp, ki, kd):
        """
        Set PID gains.

        Args:
            kp: Proportional gain (float or fixed-point)
            ki: Integral gain (float or fixed-point)
            kd: Derivative gain (float or fixed-point)
        """
        # Convert to fixed-point format (Q24.8)
        kp_bytes = fixed_to_bytes(kp)
        ki_bytes = fixed_to_bytes(ki)
        kd_bytes = fixed_to_bytes(kd)

        # Combine all gains into one buffer
        data = kp_bytes + ki_bytes + kd_bytes

        # Send via I2C
        try:
            buffer = bytearray([Command.SET_PID_GAIN_CL_MOTOR, self.instance])
            buffer.extend(data)
            self.comm.i2c.writeto(self.comm.address, buffer)
        except OSError as e:
            print(f"PID set gains error: {e}")

    def reset_gains(self):
        """Reset PID gains to default values."""
        self.comm.write_register(Command.RESET_PID_GAIN_CL_MOTOR, self.instance, 0)

    def set_control_mode(self, mode):
        """
        Set control mode.

        Args:
            mode: Control mode (ControlMode.OPEN_LOOP, VELOCITY, or POSITION)
        """
        self.comm.write_register(Command.SET_CONTROL_MODE_CL_MOTOR, self.instance, mode)

    def set_setpoint(self, target_type, target):
        """
        Set control setpoint.

        Args:
            target_type: Target type (TargetType.TARGET_POSITION or TARGET_VELOCITY)
            target: Target value (position count or velocity counts/sec)
        """
        if target_type == TargetType.TARGET_POSITION:
            self.comm.write_register(Command.SET_POSITION_SETPOINT_CL_MOTOR, self.instance, target)
        elif target_type == TargetType.TARGET_VELOCITY:
            if target == 0:
                # Fix target = 0 issue in PID_VELOCITY mode
                self.comm.write_register(Command.SET_PWM_DUTY_CYCLE_DC_MOTOR, self.instance, 0)
            else:
                self.comm.write_register(Command.SET_VELOCITY_SETPOINT_CL_MOTOR, self.instance, target)

    def set_max_acceleration(self, max_accel):
        """
        Set maximum acceleration limit.

        Args:
            max_accel: Maximum acceleration (counts/sec²)
        """
        self.comm.write_register(Command.SET_MAX_ACCELERATION_CL_MOTOR, self.instance, max_accel)

    def set_max_velocity(self, max_velocity):
        """
        Set maximum velocity limit.

        Args:
            max_velocity: Maximum velocity (counts/sec)
        """
        self.comm.write_register(Command.SET_MAX_VELOCITY_CL_MOTOR, self.instance, max_velocity)

    def set_limits(self, min_duty, max_duty):
        """
        Set duty cycle limits.

        Args:
            min_duty: Minimum duty cycle (-100 to 0)
            max_duty: Maximum duty cycle (0 to 100)
        """
        # Pack limits: (minDuty << 16) | maxDuty
        combined = ((min_duty & 0xFFFF) << 16) | (max_duty & 0xFFFF)
        self.comm.write_register(Command.SET_MIN_MAX_DUTY_CYCLE_CL_MOTOR, self.instance, combined)

    def get_gains(self):
        """
        Get current PID gains.

        Returns:
            Tuple of (kp, ki, kd) as floats or None on failure
        """
        try:
            # Request PID gains (12 bytes: 3 x 4-byte fixed-point values)
            buffer = bytearray([Command.GET_PID_VAL, self.instance])
            self.comm.i2c.writeto(self.comm.address, buffer)

            # Read response (1 byte IRQ status + 12 bytes data)
            response = self.comm.i2c.readfrom(self.comm.address, 13)

            # Extract gains (skip IRQ byte)
            data = response[1:]
            kp_fixed = bytes_to_fixed(data[0:4])
            ki_fixed = bytes_to_fixed(data[4:8])
            kd_fixed = bytes_to_fixed(data[8:12])

            # Convert to float
            kp = fixed_to_float(kp_fixed)
            ki = fixed_to_float(ki_fixed)
            kd = fixed_to_float(kd_fixed)

            return (kp, ki, kd)
        except OSError as e:
            print(f"PID get gains error: {e}")
            return None


class PIDLocal:
    """
    PID controller with local computation (d21 namespace equivalent).
    PID computation runs locally on the main board.
    """

    _next_instance = 0

    # Default PID gains
    KP_DEFAULT = 5000.0
    KI_DEFAULT = 100.0
    KD_DEFAULT = 0.0

    def __init__(self, encoder, motor, period_ms_velo=10, period_ms_pos=10):
        """
        Initialize local PID controller.

        Args:
            encoder: Encoder object for feedback
            motor: DCMotor object for control
            period_ms_velo: Velocity loop period in milliseconds (default 10ms)
            period_ms_pos: Position loop period in milliseconds (default 10ms)
        """
        self.instance = PIDLocal._next_instance
        PIDLocal._next_instance += 1

        self.encoder = encoder
        self.motor = motor

        # Control mode
        self.mode = ControlMode.VELOCITY

        # Setpoints
        self.target_pos = 0.0
        self.target_velo = 0.0

        # Inputs
        self.input_pos = 0.0
        self.input_velo = 0.0

        # Outputs
        self.velo_cmd = 0.0
        self.actual_duty = 0.0

        # Limits
        self.max_acceleration = 100.0
        self.max_velocity = 100.0
        self.min_duty = -100
        self.max_duty = 100

        # PID state for velocity loop
        self.velo_kp = self.KP_DEFAULT
        self.velo_ki = self.KI_DEFAULT
        self.velo_kd = self.KD_DEFAULT
        self.velo_integral = 0.0
        self.velo_prev_error = 0.0
        self.velo_period_s = period_ms_velo / 1000.0

        # PID state for position loop
        self.pos_kp = self.KP_DEFAULT
        self.pos_ki = self.KI_DEFAULT
        self.pos_kd = self.KD_DEFAULT
        self.pos_integral = 0.0
        self.pos_prev_error = 0.0
        self.pos_period_s = period_ms_pos / 1000.0

        # Timing
        self.last_velo_update = time.ticks_ms()
        self.last_pos_update = time.ticks_ms()
        self.prev_velo_cmd = 0.0

        # Running state
        self.running = False

    def set_gains(self, kp, ki, kd):
        """
        Set PID gains for both position and velocity loops.

        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
        """
        self.velo_kp = float(kp)
        self.velo_ki = float(ki)
        self.velo_kd = float(kd)
        self.pos_kp = float(kp)
        self.pos_ki = float(ki)
        self.pos_kd = float(kd)

    def reset_gains(self):
        """Reset PID gains to default values."""
        self.set_gains(self.KP_DEFAULT, self.KI_DEFAULT, self.KD_DEFAULT)

    def set_control_mode(self, mode):
        """
        Set control mode.

        Args:
            mode: Control mode (ControlMode.OPEN_LOOP, VELOCITY, or POSITION)
        """
        self.mode = mode

    def set_setpoint(self, target_type, target):
        """
        Set control setpoint.

        Args:
            target_type: Target type (TargetType.TARGET_POSITION or TARGET_VELOCITY)
            target: Target value
        """
        if target_type == TargetType.TARGET_VELOCITY:
            self.target_velo = float(target)
        elif target_type == TargetType.TARGET_POSITION:
            self.target_pos = float(target)

    def set_max_acceleration(self, max_accel):
        """Set maximum acceleration limit."""
        self.max_acceleration = float(max_accel)

    def set_max_velocity(self, max_velocity):
        """Set maximum velocity limit."""
        self.max_velocity = float(max_velocity)

    def set_limits(self, min_duty, max_duty):
        """Set duty cycle limits."""
        self.min_duty = min_duty
        self.max_duty = max_duty

    def run(self):
        """Start PID control."""
        self.running = True
        self.velo_integral = 0.0
        self.pos_integral = 0.0

    def stop(self):
        """Stop PID control."""
        self.running = False
        self.motor.set_duty(0)

    def update(self):
        """
        Update PID control loops.
        This should be called frequently (e.g., in main loop).
        """
        if not self.running:
            return

        # Read encoder values
        count = self.encoder.get_raw_count()
        velocity = self.encoder.get_count_per_second()

        if count is not None:
            self.input_pos = float(count)
        if velocity is not None:
            self.input_velo = float(velocity)

        # Position loop (if in position mode)
        if self.mode == ControlMode.POSITION:
            now = time.ticks_ms()
            if time.ticks_diff(now, self.last_pos_update) >= self.pos_period_s * 1000:
                self.last_pos_update = now

                # Compute position PID
                error = self.target_pos - self.input_pos
                self.pos_integral += error * self.pos_period_s
                derivative = (error - self.pos_prev_error) / self.pos_period_s
                self.velo_cmd = self.pos_kp * error + self.pos_ki * self.pos_integral + self.pos_kd * derivative
                self.pos_prev_error = error

                # Limit output to max velocity
                self.velo_cmd = max(-self.max_velocity, min(self.max_velocity, self.velo_cmd))

                # Apply acceleration limiting
                if (self.prev_velo_cmd - self.velo_cmd) > self.max_acceleration:
                    self.velo_cmd = self.prev_velo_cmd - self.max_acceleration
                elif (self.velo_cmd - self.prev_velo_cmd) > self.max_acceleration:
                    self.velo_cmd = self.prev_velo_cmd + self.max_acceleration

                # Update target velocity for velocity loop
                self.target_velo = self.velo_cmd
                self.prev_velo_cmd = self.velo_cmd

        # Velocity loop
        now = time.ticks_ms()
        if time.ticks_diff(now, self.last_velo_update) >= self.velo_period_s * 1000:
            self.last_velo_update = now

            # Compute velocity PID
            error = self.target_velo - self.input_velo
            self.velo_integral += error * self.velo_period_s
            derivative = (error - self.velo_prev_error) / self.velo_period_s
            self.actual_duty = self.velo_kp * error + self.velo_ki * self.velo_integral + self.velo_kd * derivative
            self.velo_prev_error = error

            # Limit output
            duty_out = int(max(self.min_duty, min(self.max_duty, self.actual_duty)))

            # Deadzone compensation
            if duty_out > 0:
                duty_out += 13
            elif duty_out < 0:
                duty_out -= 13

            # Send to motor
            self.motor.set_duty(duty_out)
