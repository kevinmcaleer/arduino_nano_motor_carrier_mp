"""
Simple BNO055 IMU Example
Displays orientation (Euler angles) in real-time.
Similar to the Arduino IMU_Test example.
"""

from machine import I2C, Pin
import time
import sys

sys.path.append('/lib/motor_carrier')
from bno055 import BNO055

# Initialize I2C and IMU
i2c = I2C(0, scl=Pin(12), sda=Pin(11), freq=100000)
imu = BNO055(i2c)

print("Initializing BNO055...")
if not imu.begin():
    print("Failed to initialize BNO055!")
    sys.exit(1)

print("BNO055 initialized successfully!")
print("Reading orientation data...\n")
print("Waiting for calibration...")

# Wait for basic calibration
while True:
    cal = imu.get_calibration_status()
    if cal['system'] >= 1:  # Wait for system to calibrate
        break
    print(f"Calibrating... Sys:{cal['system']} Gyr:{cal['gyro']} Acc:{cal['accel']} Mag:{cal['mag']}")
    time.sleep(0.5)

print("Calibrated! Streaming data...\n")

# Stream orientation data
start_time = time.ticks_ms()
try:
    while True:
        # Read Euler angles
        euler = imu.get_euler()

        if euler:
            timestamp = time.ticks_diff(time.ticks_ms(), start_time)

            print(f"Time: {timestamp:6d}ms")
            print(f"  Heading (Yaw): {euler['heading']:7.2f}°")
            print(f"  Roll:          {euler['roll']:7.2f}°")
            print(f"  Pitch:         {euler['pitch']:7.2f}°")
            print()

        time.sleep(0.1)  # 10 Hz update rate

except KeyboardInterrupt:
    print("\nStopped.")
