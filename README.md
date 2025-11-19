# Motor Carrier Library for MicroPython (ESP32)

This is a MicroPython port of the [Arduino Motor Carrier library](https://github.com/arduino-libraries/ArduinoMotorCarrier) for ESP32 microcontrollers. It provides full control of the Arduino Motor Carrier shield, including DC motors, servo motors, encoders, PID control, and battery monitoring.

## Features

- **4 DC Motors** (M1-M4)
  - M1, M2: I2C-based control via SAMD11 coprocessor
  - M3, M4: Direct GPIO/PWM control
- **4 Servo Motors** (via I2C)
- **2 Encoders** (position and velocity feedback via I2C)
- **PID Control** (position and velocity control)
  - I2C-based PID (runs on SAMD11)
  - Local PID (runs on ESP32)
- **Battery Monitoring** (voltage sensing)
- **Temperature Monitoring** (Motor Carrier internal temperature)

## Hardware Requirements

- ESP32 development board
- Arduino Motor Carrier shield
- Motors and encoders (as needed)
- Power supply (7-18V for Motor Carrier)

## Installation

1. Copy the `motor_carrier` folder to your ESP32's filesystem:
   ```
   /motor_carrier/
   ├── __init__.py
   ├── common.py
   ├── i2c_comm.py
   ├── controller.py
   ├── dc_motor.py
   ├── servo_motor.py
   ├── encoder.py
   ├── battery.py
   └── pid.py
   ```

2. Connect Motor Carrier to ESP32:
   - **SDA** → GPIO 11 (default, configurable)
   - **SCL** → GPIO 12 (default, configurable)
   - **M3 IN1** → GPIO 2 (configurable)
   - **M3 IN2** → GPIO 3 (configurable)
   - **M4 IN1** → GPIO 5 (configurable)
   - **M4 IN2** → GPIO 4 (configurable)

## Quick Start

```python
from motor_carrier import MotorCarrier

# Initialize Motor Carrier
mc = MotorCarrier()

# Begin communication
if mc.begin() == 0:
    print("Motor Carrier initialized!")
else:
    print("Failed to initialize")

# Control a DC motor
mc.M1.set_duty(50)  # 50% forward
mc.M1.set_duty(-50) # 50% reverse
mc.M1.set_duty(0)   # Stop

# Control a servo
mc.servo1.set_angle(90)  # Move to 90 degrees

# Read encoder
count = mc.encoder1.get_raw_count()
velocity = mc.encoder1.get_count_per_second()

# Monitor battery
voltage = mc.battery.get_filtered()
```

## API Reference

### MotorCarrier

Main class for Motor Carrier control.

```python
mc = MotorCarrier(i2c=None, scl=12, sda=11,
                  m3_pins=(2, 3), m4_pins=(4, 4))
```

**Parameters:**
- `i2c`: Existing I2C object (optional)
- `scl`: I2C SCL pin (default 12)
- `sda`: I2C SDA pin (default 11)
- `m3_pins`: Tuple of (pin_a, pin_b) for M3 motor
- `m4_pins`: Tuple of (pin_a, pin_b) for M4 motor

**Methods:**
- `begin(enable_charging=False)` - Initialize Motor Carrier (returns 0 on success)

**Attributes:**
- `controller` - MotorController object
- `M1, M2` - I2C DC motors
- `M3, M4` - GPIO DC motors
- `servo1, servo2, servo3, servo4` - Servo motors
- `encoder1, encoder2` - Encoders
- `battery` - Battery monitor
- `pid1, pid2` - I2C PID controllers

### DC Motors (M1, M2, M3, M4)

```python
mc.M1.set_duty(duty)         # Set speed: -100 to +100
mc.M1.set_frequency(freq)    # Set PWM frequency in Hz
```

**Parameters:**
- `duty`: Motor speed from -100 (full reverse) to +100 (full forward)
- `freq`: PWM frequency in Hz (e.g., 1000, 10000)

### Servo Motors

```python
mc.servo1.set_angle(angle)      # Set position: 0 to 180 degrees
mc.servo1.detach()              # Disable servo signal
mc.servo1.set_frequency(freq)   # Set PWM frequency
```

### Encoders

```python
count = mc.encoder1.get_raw_count()              # Get position
velocity = mc.encoder1.get_count_per_second()    # Get velocity
status = mc.encoder1.get_overflow_underflow()    # Get overflow status

mc.encoder1.reset_counter(0)                     # Reset to 0
mc.encoder1.set_irq_on_count(1000)              # Interrupt at count
mc.encoder1.set_irq_on_velocity(100, margin=2)  # Interrupt at velocity
```

### Battery Monitor

```python
raw = mc.battery.get_raw()          # Raw ADC value
converted = mc.battery.get_converted()  # Converted value
filtered = mc.battery.get_filtered()    # Filtered value (smoothed)
```

### MotorController

```python
version = mc.controller.get_fw_version()   # Firmware version
temp = mc.controller.get_temperature()     # Temperature in °C
ram = mc.controller.get_free_ram()         # Free RAM in bytes
status = mc.controller.get_irq_status()    # IRQ status

mc.controller.ping()                       # Keep-alive ping
mc.controller.reboot()                     # Reboot controller
```

### PID Control (I2C-based)

PID control running on SAMD11 coprocessor.

```python
from motor_carrier import ControlMode, TargetType

# Configure PID
mc.pid1.set_gains(kp=1.5, ki=0.1, kd=0.0)
mc.pid1.set_control_mode(ControlMode.POSITION)
mc.pid1.set_setpoint(TargetType.TARGET_POSITION, 1000)
mc.pid1.set_max_velocity(200)
mc.pid1.set_max_acceleration(100)
mc.pid1.set_limits(min_duty=-100, max_duty=100)

# Read gains
kp, ki, kd = mc.pid1.get_gains()
```

**Control Modes:**
- `ControlMode.OPEN_LOOP` - No PID control
- `ControlMode.VELOCITY` - Velocity control
- `ControlMode.POSITION` - Position control

**Target Types:**
- `TargetType.TARGET_VELOCITY` - Set velocity setpoint (counts/sec)
- `TargetType.TARGET_POSITION` - Set position setpoint (counts)

### PID Control (Local)

PID control running locally on ESP32.

```python
# Create local PID for encoder1 + motor M3
pid = mc.create_local_pid(
    encoder_num=1,
    motor_num=3,
    period_ms_velo=10,
    period_ms_pos=10
)

# Configure and run
pid.set_gains(kp=0.5, ki=0.01, kd=0.0)
pid.set_control_mode(ControlMode.VELOCITY)
pid.set_setpoint(TargetType.TARGET_VELOCITY, 100)
pid.run()

# Update loop (call frequently!)
while True:
    pid.update()  # MUST be called frequently (e.g., every 10ms)
    time.sleep(0.01)

# Stop
pid.stop()
```

**Important:** Local PID requires calling `pid.update()` frequently in your main loop!

## Examples

See the `examples/` folder for complete working examples:

- **motor_test.py** - Test all DC motors
- **servo_test.py** - Test all servos with sweep
- **encoder_test.py** - Read encoder position/velocity
- **pid_position_test.py** - Closed-loop position control (I2C PID)
- **local_pid_test.py** - Closed-loop control with local PID
- **battery_monitor.py** - Monitor battery voltage

## Pin Configuration

### Default I2C Pins
- SDA: GPIO 11
- SCL: GPIO 12

### Default GPIO Motor Pins (M3/M4) - Nano 33 IoT Compatible
- M3: GPIO 2 (IN1), GPIO 3 (IN2)
- M4: GPIO 5 (IN1), GPIO 4 (IN2)

You can customize these pins when creating the MotorCarrier object:

```python
mc = MotorCarrier(
    scl=12, sda=11,
    m3_pins=(2, 3),
    m4_pins=(5, 4)
)
```

## Differences from Arduino Library

### Namespace Changes
- **Arduino:** Uses `mc::` and `d21::` namespaces
- **MicroPython:** Uses class names `DCMotorI2C` / `DCMotorGPIO`, `PIDI2C` / `PIDLocal`

### Method Naming
- **Arduino:** camelCase (e.g., `setDuty`, `getRawCount`)
- **MicroPython:** snake_case (e.g., `set_duty`, `get_raw_count`)

### Global Objects
- **Arduino:** Automatically creates global objects (`M1`, `servo1`, etc.)
- **MicroPython:** Access via `mc.M1`, `mc.servo1`, etc.

### Fixed-Point Math
- **Arduino:** Uses `Fix16` (Q24.8 fixed-point)
- **MicroPython:** Accepts floats, converts internally to fixed-point for I2C

### PID Update Loop
- **Arduino (d21::PID):** `update()` must be called frequently
- **MicroPython (PIDLocal):** Same - `update()` must be called frequently

## Troubleshooting

### I2C Communication Errors

**Problem:** `I2C write error` or `I2C read error`

**Solutions:**
1. Check I2C connections (SDA, SCL)
2. Verify Motor Carrier is powered (7-18V)
3. Check I2C pull-up resistors (usually built-in on ESP32)
4. Try different I2C pins
5. Reduce I2C frequency: `I2CComm(freq=50000)`

### Motor Carrier Not Responding

**Problem:** `begin()` returns 1 or firmware version is "0"

**Solutions:**
1. Check Motor Carrier power supply
2. Verify SAMD11 coprocessor is programmed
3. Press reset button on Motor Carrier
4. Try `mc.controller.reboot()`

### Motors Not Moving

**Problem:** `set_duty()` doesn't move motor

**Solutions:**
1. Check motor power supply (7-18V)
2. Verify motor connections (M1-M4 terminals)
3. Check if motor is in PID mode (set to OPEN_LOOP)
4. For M3/M4, verify GPIO pin configuration
5. Test with higher duty cycle (e.g., 100)

### Encoders Not Working

**Problem:** Encoder always returns 0 or None

**Solutions:**
1. Check encoder connections to Motor Carrier
2. Verify encoder power (usually 5V)
3. Test encoder with multimeter (A/B channels should toggle)
4. Reset encoder: `mc.encoder1.reset_counter(0)`

### PID Not Controlling

**Problem:** PID doesn't control motor position/velocity

**Solutions:**
1. Verify encoder is working
2. Check PID gains (may need tuning)
3. Ensure control mode is set correctly
4. For local PID, verify `update()` is called frequently
5. Check motor duty limits are reasonable (-100 to 100)
6. Start with velocity control before trying position control

## License

This MicroPython library maintains the same LGPL 2.1 license as the original Arduino library.

```
Copyright (c) 2018-2019 Arduino AG.  All rights reserved.
Copyright (c) 2025 MicroPython Port

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
```

## Contributing

Contributions are welcome! Please test thoroughly on ESP32 hardware before submitting pull requests.

## Credits

- Original Arduino library: [ArduinoMotorCarrier](https://github.com/arduino-libraries/ArduinoMotorCarrier)
- MicroPython port: Converted from C++/Arduino to MicroPython for ESP32

## Support

For issues specific to the MicroPython port, please check:
1. This README troubleshooting section
2. Example scripts in `examples/` folder
3. Original Arduino library documentation

For hardware-specific issues with the Motor Carrier, refer to the [original Arduino library](https://github.com/arduino-libraries/ArduinoMotorCarrier).
