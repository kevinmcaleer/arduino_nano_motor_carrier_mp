"""
Local PID Control Test Example
Tests local PID control (running on ESP32) for motors M3/M4.

This is equivalent to the d21::PID implementation in the Arduino library.
The PID computation runs locally on the ESP32 instead of the SAMD11.
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

# Create local PID controller for encoder1 + motor M3
print("Creating local PID controller...")
pid = mc.create_local_pid(
    encoder_num=1,      # Use encoder 1
    motor_num=3,        # Use motor M3 (GPIO controlled)
    period_ms_velo=10,  # 10ms velocity loop
    period_ms_pos=10    # 10ms position loop
)

# Configure PID
print("Configuring PID...")
pid.set_gains(0.5, 0.01, 0.0)  # Lower gains for local PID
pid.set_control_mode(ControlMode.VELOCITY)
pid.set_max_velocity(100)
pid.set_max_acceleration(50)
pid.set_limits(-100, 100)

# Start PID
print("Starting PID control...")
pid.run()

print("\n=== Local PID Velocity Control Test ===")
print("Motor M3 will track different velocity setpoints.\n")

# Test different velocities
velocities = [0, 50, 100, 150, 100, 50, 0, -50, -100, -50, 0]

for target_vel in velocities:
    print(f"Target velocity: {target_vel} counts/sec")
    pid.set_setpoint(TargetType.TARGET_VELOCITY, target_vel)

    # Monitor for 2 seconds, updating PID frequently
    start_time = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start_time) < 2000:
        # IMPORTANT: Call update() frequently for PID to work
        pid.update()

        # Display status every 200ms
        if time.ticks_diff(time.ticks_ms(), start_time) % 200 < 20:
            count = mc.encoder1.get_raw_count()
            velocity = mc.encoder1.get_count_per_second()

            if count is not None and velocity is not None:
                error = target_vel - velocity
                print(f"  Velocity: {velocity:6d} | Target: {target_vel:6d} | Error: {error:6d} | Position: {count:6d}")

        time.sleep(0.01)  # Small delay, but keep updating PID

    print()

# Stop PID
print("Stopping PID control...")
pid.stop()

print("=== Test Complete ===")
