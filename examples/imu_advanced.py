"""
Advanced BNO055 IMU Example
Demonstrates quaternions, linear acceleration, and gravity vector.
"""

from machine import I2C, Pin
import time
import math
import sys

sys.path.append('/lib/motor_carrier')
from bno055 import BNO055

# Initialize I2C and IMU
i2c = I2C(0, scl=Pin(12), sda=Pin(11), freq=100000)
imu = BNO055(i2c)

print("=" * 60)
print("Advanced BNO055 IMU Features")
print("=" * 60)

if not imu.begin():
    print("Failed to initialize BNO055!")
    sys.exit(1)

print("BNO055 initialized!\n")

# Display mode selection
print("Select data to display:")
print("1. Quaternion (orientation as quaternion)")
print("2. Linear Acceleration (without gravity)")
print("3. Gravity Vector")
print("4. All Sensors")
print()

mode = 4  # Default to all sensors

print("Displaying all sensor data (mode 4)...")
print("Press Ctrl+C to stop\n")

try:
    while True:
        cal = imu.get_calibration_status()

        # Quaternion
        quat = imu.get_quaternion()
        if quat and mode in [1, 4]:
            print(f"Quaternion - W:{quat['w']:6.3f} X:{quat['x']:6.3f} "
                  f"Y:{quat['y']:6.3f} Z:{quat['z']:6.3f}")

        # Euler (for reference)
        euler = imu.get_euler()
        if euler and mode == 4:
            print(f"Euler      - H:{euler['heading']:7.2f}° "
                  f"R:{euler['roll']:7.2f}° P:{euler['pitch']:7.2f}°")

        # Linear acceleration (gravity removed)
        linear = imu.get_linear_acceleration()
        if linear and mode in [2, 4]:
            magnitude = math.sqrt(linear['x']**2 + linear['y']**2 + linear['z']**2)
            print(f"Linear Acc - X:{linear['x']:6.2f} Y:{linear['y']:6.2f} "
                  f"Z:{linear['z']:6.2f} |{magnitude:6.2f}| m/s²")

        # Gravity vector
        gravity = imu.get_gravity()
        if gravity and mode in [3, 4]:
            print(f"Gravity    - X:{gravity['x']:6.2f} Y:{gravity['y']:6.2f} "
                  f"Z:{gravity['z']:6.2f} m/s²")

        # Raw accelerometer
        accel = imu.get_accelerometer()
        if accel and mode == 4:
            print(f"Accel (raw)- X:{accel['x']:6.2f} Y:{accel['y']:6.2f} "
                  f"Z:{accel['z']:6.2f} m/s²")

        # Gyroscope
        gyro = imu.get_gyroscope()
        if gyro and mode == 4:
            print(f"Gyro       - X:{gyro['x']:7.2f} Y:{gyro['y']:7.2f} "
                  f"Z:{gyro['z']:7.2f} °/s")

        # Magnetometer
        mag = imu.get_magnetometer()
        if mag and mode == 4:
            heading = math.atan2(mag['y'], mag['x']) * 180 / math.pi
            if heading < 0:
                heading += 360
            print(f"Mag        - X:{mag['x']:7.2f} Y:{mag['y']:7.2f} "
                  f"Z:{mag['z']:7.2f} µT (Heading: {heading:.1f}°)")

        # Calibration status
        print(f"Calibration- Sys:{cal['system']} Gyr:{cal['gyro']} "
              f"Acc:{cal['accel']} Mag:{cal['mag']}")

        print()  # Blank line between readings
        time.sleep(0.2)  # 5 Hz update

except KeyboardInterrupt:
    print("\n\nStopped.")

print("\n" + "=" * 60)
print("Advanced IMU Test Complete")
print("=" * 60)
