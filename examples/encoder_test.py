"""
Encoder Test Example
Tests encoder reading with Motor Carrier.
Runs a motor and displays encoder position and velocity.
"""

from motor_carrier import MotorCarrier
import time

# Initialize Motor Carrier
mc = MotorCarrier()

# Initialize
print("Initializing Motor Carrier...")
if mc.begin() != 0:
    print("ERROR: Failed to initialize Motor Carrier!")
else:
    print("Motor Carrier initialized successfully!")

# Reset encoders to zero
print("\nResetting encoders to zero...")
mc.encoder1.reset_counter(0)
mc.encoder2.reset_counter(0)
time.sleep(0.1)

# Verify reset
count1 = mc.encoder1.get_raw_count()
count2 = mc.encoder2.get_raw_count()
print(f"Encoder 1: {count1}")
print(f"Encoder 2: {count2}")

print("\n=== Encoder Test ===")
print("Motor M1 will run at different speeds.")
print("Encoder 1 position and velocity will be displayed.\n")

# Test different motor speeds
speeds = [30, 50, 75, 100, 75, 50, 30, 0, -30, -50, -30, 0]

for speed in speeds:
    print(f"Motor speed: {speed}%")
    mc.M1.set_duty(speed)

    # Read encoder for 2 seconds
    for i in range(10):
        count = mc.encoder1.get_raw_count()
        velocity = mc.encoder1.get_count_per_second()

        if count is not None and velocity is not None:
            print(f"  Count: {count:6d}  |  Velocity: {velocity:6d} counts/sec")
        else:
            print("  Error reading encoder")

        time.sleep(0.2)

    print()

# Stop motor
mc.M1.set_duty(0)

print("=== Test Complete ===")
print("\nFinal encoder readings:")
count1 = mc.encoder1.get_raw_count()
count2 = mc.encoder2.get_raw_count()
print(f"Encoder 1: {count1}")
print(f"Encoder 2: {count2}")

# Test overflow/underflow status
status1 = mc.encoder1.get_overflow_underflow()
status2 = mc.encoder2.get_overflow_underflow()
print(f"\nEncoder 1 overflow/underflow status: {status1}")
print(f"Encoder 2 overflow/underflow status: {status2}")
