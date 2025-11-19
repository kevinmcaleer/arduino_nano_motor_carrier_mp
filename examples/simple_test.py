"""
Simple Motor Carrier Connection Test
Tests basic I2C communication with Motor Carrier.
"""

from motor_carrier import MotorCarrier
import time

print("=" * 50)
print("Motor Carrier Connection Test")
print("=" * 50)

# Initialize Motor Carrier with explicit pins
print("\n1. Creating MotorCarrier object...")
try:
    mc = MotorCarrier(scl=12, sda=11)
    print("   ✓ MotorCarrier object created")
except Exception as e:
    print(f"   ✗ Error: {e}")
    import sys
    sys.exit(1)

# Test I2C connection
print("\n2. Testing I2C connection...")
try:
    from machine import I2C, Pin
    i2c = I2C(0, scl=Pin(12), sda=Pin(11), freq=100000)
    devices = i2c.scan()
    print(f"   I2C devices found: {[hex(d) for d in devices]}")

    if 0x66 in devices:
        print("   ✓ Motor Carrier found at address 0x66")

        # Try a raw version read to see what we get
        print("   Testing raw version read...")
        try:
            i2c.writeto(0x66, bytes([0x01, 0x00]))  # GET_VERSION command
            response = i2c.readfrom(0x66, 16)
            print(f"   Raw response: {response}")
            print(f"   Hex: {' '.join('%02x' % b for b in response)}")
        except Exception as e:
            print(f"   Raw read error: {e}")
    else:
        print("   ✗ Motor Carrier NOT found at expected address 0x66")
        print("   Check your wiring:")
        print("     - SDA → GPIO 11")
        print("     - SCL → GPIO 12")
        print("     - Motor Carrier power (7-18V)")
        print("   Tip: Make sure the Motor Carrier LED is on")
except Exception as e:
    print(f"   ✗ I2C Error: {e}")
    import sys
    sys.print_exception(e)

# Initialize Motor Carrier
print("\n3. Initializing Motor Carrier...")
try:
    result = mc.begin()
    if result == 0:
        print("   ✓ Motor Carrier initialized successfully!")
    else:
        print(f"   ✗ Initialization failed (error code: {result})")
        print("   Possible issues:")
        print("     - Motor Carrier not powered")
        print("     - I2C connections incorrect")
        print("     - SAMD11 firmware not loaded")
except Exception as e:
    print(f"   ✗ Error during initialization: {e}")
    import sys
    sys.print_exception(e)
    import sys
    sys.exit(1)

# Get firmware version
print("\n4. Reading firmware version...")
try:
    version = mc.controller.get_fw_version()
    print(f"   Firmware version: {version}")
except Exception as e:
    print(f"   ✗ Error reading version: {e}")

# Get temperature
print("\n5. Reading temperature...")
try:
    temp = mc.controller.get_temperature()
    if temp is not None:
        print(f"   Temperature: {temp:.1f}°C")
        print("   ✓ Temperature sensor working")
    else:
        print("   ⚠️  Temperature not available (firmware may not support it)")
except Exception as e:
    print(f"   ✗ Error: {e}")

# Test battery reading
print("\n6. Testing battery sensor...")
try:
    status = mc.battery.get_status()

    if status:
        print(f"   Raw ADC: {status['raw']}")
        print(f"   Voltage: {status['voltage']:.2f}V")
        print(f"   Status: {status['status']}")
        print(f"   Message: {status['message']}")
        print(f"   Firmware converted: {status['converted']}")
        print(f"   Firmware filtered: {status['filtered']}")
        print("   ✓ Battery sensor working")

        if status['voltage'] < 6.0:
            print("\n   ⚠️  WARNING: No battery detected or critically low!")
            print("   Connect a LiPo battery (7-18V) to the Motor Carrier")
    else:
        print("   ✗ Battery sensor not responding")
except Exception as e:
    print(f"   ✗ Error: {e}")
    import sys
    sys.print_exception(e)

print("\n" + "=" * 50)
print("Test complete!")
print("=" * 50)
