"""
Servo Motor Test Example
Tests all 4 servo motors with Motor Carrier.
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

# Test all servos
servos = [
    ("Servo 1", mc.servo1),
    ("Servo 2", mc.servo2),
    ("Servo 3", mc.servo3),
    ("Servo 4", mc.servo4)
]

print("\n=== Servo Test ===")
print("Each servo will move to 0°, 90°, and 180°.\n")

for servo_name, servo in servos:
    print(f"Testing {servo_name}...")

    # Move to 0 degrees
    print(f"  {servo_name}: 0°")
    servo.set_angle(0)
    time.sleep(1)

    # Move to 90 degrees (center)
    print(f"  {servo_name}: 90°")
    servo.set_angle(90)
    time.sleep(1)

    # Move to 180 degrees
    print(f"  {servo_name}: 180°")
    servo.set_angle(180)
    time.sleep(1)

    # Back to center
    print(f"  {servo_name}: 90°")
    servo.set_angle(90)
    time.sleep(1)

    # Detach servo
    print(f"  {servo_name}: Detached")
    servo.detach()
    time.sleep(1)

    print(f"{servo_name} test complete!\n")

print("=== Servo Test Complete ===")

# Sweep test - all servos together
print("\n=== Synchronized Sweep Test ===")
print("All servos will sweep from 0° to 180° and back.\n")

# Attach all servos at center position
for _, servo in servos:
    servo.set_angle(90)
time.sleep(1)

# Sweep 0 to 180
print("Sweeping 0° to 180°...")
for angle in range(0, 181, 5):
    for _, servo in servos:
        servo.set_angle(angle)
    time.sleep(0.05)

time.sleep(1)

# Sweep 180 to 0
print("Sweeping 180° to 0°...")
for angle in range(180, -1, -5):
    for _, servo in servos:
        servo.set_angle(angle)
    time.sleep(0.05)

# Center all servos
print("Centering all servos...")
for _, servo in servos:
    servo.set_angle(90)

print("\n=== All Tests Complete ===")
