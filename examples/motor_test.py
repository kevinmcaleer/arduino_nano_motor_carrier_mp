"""
DC Motor Test Example
Tests all 4 DC motors (M1-M4) with Motor Carrier.

M1, M2: Controlled via I2C (SAMD11 coprocessor)
M3, M4: Controlled via GPIO/PWM (direct control)
"""

from motor_carrier import MotorCarrier
import time

# Initialize Motor Carrier
# Using Nano 33 IoT pin defaults (adjust if using custom wiring)
mc = MotorCarrier(
    scl=12,      # I2C SCL pin
    sda=11,      # I2C SDA pin
    m3_pins=(2, 3),   # M3 control pins (Nano 33 IoT: pins 2, 3)
    m4_pins=(5, 4)    # M4 control pins (Nano 33 IoT: pins 5, 4)
)

# Initialize
print("Initializing Motor Carrier...")
if mc.begin() != 0:
    print("ERROR: Failed to initialize Motor Carrier!")
    print("Check I2C connections and Motor Carrier power.")
else:
    print("Motor Carrier initialized successfully!")
    version = mc.controller.get_fw_version()
    print(f"Firmware version: {version}")

# Test each motor
motors = [
    ("M1", mc.M1),
    ("M2", mc.M2),
    ("M3", mc.M3),
    ("M4", mc.M4)
]

print("\n=== Motor Test ===")
print("Each motor will run forward, reverse, and stop.\n")

for motor_name, motor in motors:
    print(f"Testing {motor_name}...")

    # Forward at 50% speed
    print(f"  {motor_name}: Forward 50%")
    motor.set_duty(50)
    time.sleep(2)

    # Stop
    print(f"  {motor_name}: Stop")
    motor.set_duty(0)
    time.sleep(1)

    # Reverse at 50% speed
    print(f"  {motor_name}: Reverse 50%")
    motor.set_duty(-50)
    time.sleep(2)

    # Stop
    print(f"  {motor_name}: Stop")
    motor.set_duty(0)
    time.sleep(1)

    # Full speed forward
    print(f"  {motor_name}: Forward 100%")
    motor.set_duty(100)
    time.sleep(2)

    # Stop
    print(f"  {motor_name}: Stop")
    motor.set_duty(0)
    time.sleep(1)

    print(f"{motor_name} test complete!\n")

print("=== Motor Test Complete ===")

# Keep pinging to maintain communication
print("\nKeeping Motor Carrier alive (Ctrl+C to stop)...")
try:
    while True:
        mc.controller.ping()
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopping...")
    # Stop all motors
    mc.M1.set_duty(0)
    mc.M2.set_duty(0)
    mc.M3.set_duty(0)
    mc.M4.set_duty(0)
    print("All motors stopped.")
