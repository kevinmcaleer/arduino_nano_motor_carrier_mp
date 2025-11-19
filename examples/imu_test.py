"""
BNO055 IMU Test Example
Tests the 9-axis IMU (accelerometer, gyroscope, magnetometer) on Motor Carrier.
"""

from machine import I2C, Pin
import time

# Import BNO055 class
import sys
sys.path.append('/lib/motor_carrier')
from bno055 import BNO055, OPERATION_MODE_NDOF

print("=" * 60)
print("BNO055 IMU Test")
print("=" * 60)

# Initialize I2C
print("\n1. Initializing I2C...")
i2c = I2C(0, scl=Pin(12), sda=Pin(11), freq=100000)

# Scan for devices
devices = i2c.scan()
print(f"   I2C devices found: {[hex(d) for d in devices]}")

if 0x28 in devices:
    print("   ✓ BNO055 found at address 0x28")
else:
    print("   ✗ BNO055 not found!")
    print("   Check I2C connections")
    import sys
    sys.exit(1)

# Initialize BNO055
print("\n2. Initializing BNO055 IMU...")
imu = BNO055(i2c)

if imu.begin():
    print("   ✓ BNO055 initialized successfully")
else:
    print("   ✗ BNO055 initialization failed")
    import sys
    sys.exit(1)

# Check system status
print("\n3. System Status...")
status = imu.get_system_status()
if status:
    print(f"   Status: {status['status_msg']}")
    print(f"   Error: {status['error_msg']}")

# Get temperature
print("\n4. Temperature...")
temp = imu.get_temperature()
print(f"   Temperature: {temp}°C")

# Calibration status
print("\n5. Calibration Status (move sensor to calibrate)...")
print("   Gyro: Move sensor in various orientations")
print("   Accel: Place sensor in 6 different positions")
print("   Mag: Move sensor in figure-8 pattern")
print()

for i in range(20):
    cal = imu.get_calibration_status()
    sys_cal = '█' * cal['system'] + '░' * (3 - cal['system'])
    gyr_cal = '█' * cal['gyro'] + '░' * (3 - cal['gyro'])
    acc_cal = '█' * cal['accel'] + '░' * (3 - cal['accel'])
    mag_cal = '█' * cal['mag'] + '░' * (3 - cal['mag'])

    print(f"   Sys:{sys_cal}({cal['system']}) Gyr:{gyr_cal}({cal['gyro']}) "
          f"Acc:{acc_cal}({cal['accel']}) Mag:{mag_cal}({cal['mag']})", end="")

    if imu.is_calibrated():
        print(" ✓ FULLY CALIBRATED")
        break
    else:
        print()

    time.sleep(0.5)

print("\n6. Reading Sensor Data...")
print("   Press Ctrl+C to stop\n")

try:
    while True:
        # Euler angles (orientation)
        euler = imu.get_euler()
        if euler:
            print(f"Euler - Heading:{euler['heading']:7.2f}° Roll:{euler['roll']:7.2f}° Pitch:{euler['pitch']:7.2f}°")

        # Accelerometer
        accel = imu.get_accelerometer()
        if accel:
            print(f"Accel - X:{accel['x']:6.2f} Y:{accel['y']:6.2f} Z:{accel['z']:6.2f} m/s²")

        # Gyroscope
        gyro = imu.get_gyroscope()
        if gyro:
            print(f"Gyro  - X:{gyro['x']:6.2f} Y:{gyro['y']:6.2f} Z:{gyro['z']:6.2f} °/s")

        # Magnetometer
        mag = imu.get_magnetometer()
        if mag:
            print(f"Mag   - X:{mag['x']:6.2f} Y:{mag['y']:6.2f} Z:{mag['z']:6.2f} µT")

        # Calibration status
        cal = imu.get_calibration_status()
        print(f"Cal   - Sys:{cal['system']} Gyr:{cal['gyro']} Acc:{cal['accel']} Mag:{cal['mag']}")

        print()  # Blank line
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\n\nTest stopped.")

print("\n" + "=" * 60)
print("IMU Test Complete")
print("=" * 60)
