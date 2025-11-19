"""
PID Position Control Test Example
Tests closed-loop position control using encoder feedback.

This example uses I2C-based PID control (runs on SAMD11 coprocessor).
"""

from motor_carrier import MotorCarrier, ControlMode, TargetType
import time

# Initialize Motor Carrier
mc = MotorCarrier()

# Initialize
print("Initializing Motor Carrier...")
if mc.begin() != 0:
    print("ERROR: Failed to initialize Motor Carrier!")
else:
    print("Motor Carrier initialized successfully!")

# Reset encoder
print("\nResetting encoder to zero...")
mc.encoder1.reset_counter(0)
time.sleep(0.1)

# Configure PID controller
print("Configuring PID controller...")

# Set PID gains (kp, ki, kd)
# These values may need tuning for your specific motor/encoder
mc.pid1.set_gains(1.5, 0.1, 0.0)

# Set control mode to position
mc.pid1.set_control_mode(ControlMode.POSITION)

# Set velocity and acceleration limits
mc.pid1.set_max_velocity(200)        # Max 200 counts/sec
mc.pid1.set_max_acceleration(100)    # Max 100 counts/sec²

# Set duty cycle limits
mc.pid1.set_limits(-100, 100)        # -100% to +100%

print("PID configured!")

# Test position control
positions = [0, 500, 1000, 500, 0, -500, -1000, -500, 0]

print("\n=== PID Position Control Test ===")
print("Motor will move to different positions using closed-loop control.\n")

for target_pos in positions:
    print(f"Target position: {target_pos}")
    mc.pid1.set_setpoint(TargetType.TARGET_POSITION, target_pos)

    # Monitor position for 3 seconds
    for i in range(15):
        count = mc.encoder1.get_raw_count()
        velocity = mc.encoder1.get_count_per_second()

        if count is not None and velocity is not None:
            error = target_pos - count
            print(f"  Position: {count:6d} | Target: {target_pos:6d} | Error: {error:6d} | Velocity: {velocity:6d}")
        else:
            print("  Error reading encoder")

        time.sleep(0.2)

    print()

# Stop PID control
print("Stopping PID control...")
mc.pid1.set_control_mode(ControlMode.OPEN_LOOP)
mc.M1.set_duty(0)

print("=== Test Complete ===")
print("\nFinal position:")
count = mc.encoder1.get_raw_count()
print(f"Encoder 1: {count}")
