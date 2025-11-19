# BNO055 IMU Support

The Motor Carrier includes a **BNO055 9-axis IMU** (Inertial Measurement Unit) with:
- 3-axis accelerometer
- 3-axis gyroscope
- 3-axis magnetometer
- Built-in sensor fusion for orientation tracking

## Quick Start

### 1. Upload the BNO055 Driver

Copy `bno055.py` to your ESP32:
```python
# Upload to /lib/motor_carrier/bno055.py
```

### 2. Basic Usage

```python
from machine import I2C, Pin
from bno055 import BNO055

# Initialize I2C and IMU
i2c = I2C(0, scl=Pin(12), sda=Pin(11), freq=100000)
imu = BNO055(i2c)

# Initialize the sensor
imu.begin()

# Read orientation (Euler angles)
euler = imu.get_euler()
print(f"Heading: {euler['heading']}°")
print(f"Roll: {euler['roll']}°")
print(f"Pitch: {euler['pitch']}°")

# Read accelerometer
accel = imu.get_accelerometer()
print(f"Acceleration: X={accel['x']} Y={accel['y']} Z={accel['z']} m/s²")
```

## API Reference

### Initialization

```python
imu = BNO055(i2c, address=0x28)
imu.begin()  # Returns True on success
```

### Calibration

The BNO055 needs calibration for accurate readings:

```python
# Get calibration status (0-3, where 3 is fully calibrated)
cal = imu.get_calibration_status()
print(f"System: {cal['system']}")
print(f"Gyro: {cal['gyro']}")
print(f"Accel: {cal['accel']}")
print(f"Mag: {cal['mag']}")

# Check if fully calibrated
if imu.is_calibrated():
    print("Fully calibrated!")
```

**Calibration Tips:**
- **Gyroscope**: Keep sensor stationary for a few seconds
- **Accelerometer**: Place sensor in 6 different orientations (all faces up/down)
- **Magnetometer**: Move sensor in figure-8 pattern away from magnetic interference

### Orientation (Sensor Fusion)

```python
# Euler angles (heading, roll, pitch)
euler = imu.get_euler()
# Returns: {'heading': 0-360°, 'roll': ±180°, 'pitch': ±90°}

# Quaternion (more stable for 3D rotations)
quat = imu.get_quaternion()
# Returns: {'w': w, 'x': x, 'y': y, 'z': z}
```

### Raw Sensor Data

```python
# Accelerometer (m/s²)
accel = imu.get_accelerometer()
# Returns: {'x': x, 'y': y, 'z': z}

# Gyroscope (degrees/second)
gyro = imu.get_gyroscope()
# Returns: {'x': x, 'y': y, 'z': z}

# Magnetometer (micro-Tesla)
mag = imu.get_magnetometer()
# Returns: {'x': x, 'y': y, 'z': z}
```

### Processed Data

```python
# Linear acceleration (gravity removed)
linear = imu.get_linear_acceleration()
# Returns: {'x': x, 'y': y, 'z': z} in m/s²

# Gravity vector
gravity = imu.get_gravity()
# Returns: {'x': x, 'y': y, 'z': z} in m/s²
```

### Operation Modes

```python
from bno055 import (
    OPERATION_MODE_CONFIG,      # Configuration mode
    OPERATION_MODE_ACCONLY,     # Accelerometer only
    OPERATION_MODE_MAGONLY,     # Magnetometer only
    OPERATION_MODE_GYRONLY,     # Gyroscope only
    OPERATION_MODE_ACCMAG,      # Accel + Mag
    OPERATION_MODE_ACCGYRO,     # Accel + Gyro
    OPERATION_MODE_MAGGYRO,     # Mag + Gyro
    OPERATION_MODE_AMG,         # All sensors (no fusion)
    OPERATION_MODE_IMUPLUS,     # Accel + Gyro fusion (no mag)
    OPERATION_MODE_COMPASS,     # Mag + Accel fusion
    OPERATION_MODE_M4G,         # Accel + Mag fusion
    OPERATION_MODE_NDOF_FMC_OFF,# 9-axis fusion (fast mag calibration off)
    OPERATION_MODE_NDOF         # 9-axis fusion (default)
)

# Set mode
imu.set_mode(OPERATION_MODE_NDOF)
```

### System Status

```python
# Get system status and error codes
status = imu.get_system_status()
print(status['status_msg'])  # e.g., "Sensor Fusion Running"
print(status['error_msg'])   # e.g., "No Error"

# Get temperature
temp = imu.get_temperature()  # In degrees Celsius
```

## Examples

### Example 1: Simple Orientation (imu_simple.py)

Displays heading, roll, and pitch similar to Arduino example:
```python
import examples.imu_simple
```

**Output:**
```
Time:   1000ms
  Heading (Yaw):   45.25°
  Roll:             5.12°
  Pitch:           -2.34°
```

### Example 2: Full Sensor Test (imu_test.py)

Comprehensive test showing calibration and all sensor readings:
```python
import examples.imu_test
```

**Features:**
- Displays calibration progress
- Shows all sensor data (accel, gyro, mag, euler)
- Real-time calibration status bars

### Example 3: Advanced Features (imu_advanced.py)

Demonstrates quaternions, linear acceleration, and gravity vector:
```python
import examples.imu_advanced
```

**Features:**
- Quaternion representation
- Linear acceleration (gravity removed)
- Gravity vector
- Computed magnetic heading

## Common Use Cases

### 1. Robot Orientation

```python
while True:
    euler = imu.get_euler()
    heading = euler['heading']

    # Use heading for navigation
    if heading < 90:
        print("Facing East")
    # ... etc
```

### 2. Motion Detection

```python
while True:
    linear = imu.get_linear_acceleration()
    magnitude = (linear['x']**2 + linear['y']**2 + linear['z']**2)**0.5

    if magnitude > 2.0:  # 2 m/s² threshold
        print("Significant motion detected!")
```

### 3. Tilt Detection

```python
while True:
    euler = imu.get_euler()

    if abs(euler['roll']) > 45 or abs(euler['pitch']) > 45:
        print("Warning: Device tilted!")
```

### 4. Compass Heading

```python
while True:
    euler = imu.get_euler()
    heading = euler['heading']

    # Convert to cardinal directions
    directions = ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']
    index = int((heading + 22.5) / 45) % 8
    print(f"Heading: {heading:.1f}° ({directions[index]})")
```

## Troubleshooting

### IMU Not Detected
- **Check I2C scan**: Should show device at 0x28
- **Check wiring**: SDA → GPIO 11, SCL → GPIO 12
- **Power**: Ensure Motor Carrier is powered

### Poor Calibration
- **Gyro**: Keep completely still during calibration
- **Accel**: Move through all 6 orientations slowly
- **Mag**: Avoid metal objects and magnetic fields
- **Environment**: Calibrate away from motors/electronics

### Erratic Readings
- **Magnetic interference**: Keep away from motors when running
- **Vibration**: Mount IMU firmly
- **Update rate**: Don't read too fast (< 100Hz recommended)

### Calibration Lost
- BNO055 loses calibration on power cycle
- Consider saving/loading calibration profile (advanced)
- Re-calibrate after each power-on

## Technical Details

### I2C Address
- Default: **0x28**
- Alternative: 0x29 (if COM3 pin is high)

### Data Rates
- Accelerometer: up to 100 Hz
- Gyroscope: up to 100 Hz
- Magnetometer: up to 20 Hz
- Fusion output: up to 100 Hz

### Units
- **Euler angles**: Degrees (heading 0-360°, roll/pitch ±180°/±90°)
- **Quaternion**: Unitless normalized values
- **Acceleration**: m/s²
- **Angular velocity**: degrees/second
- **Magnetic field**: micro-Tesla (µT)

### Coordinate System
- **X**: Forward (Roll axis)
- **Y**: Left (Pitch axis)
- **Z**: Up (Yaw axis)

## Integration with Motor Carrier

You can use the IMU with motor control for advanced robotics:

```python
from motor_carrier import MotorCarrier

mc = MotorCarrier()
mc.begin()

# Access IMU
euler = mc.imu.get_euler()

# Use orientation for motor control
heading_error = target_heading - euler['heading']
mc.M1.set_duty(int(heading_error * 0.5))  # Proportional control
```

## References

- [BNO055 Datasheet](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
- [BNO055 Register Map](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
- Motor Carrier Documentation

## License

This BNO055 driver is part of the Motor Carrier MicroPython library.
LGPL 2.1 License - Same as Arduino Motor Carrier library.
