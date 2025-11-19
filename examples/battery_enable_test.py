"""
Battery Enable Test
Try enabling battery charging/PMIC to see if that helps read battery voltage.
"""

from motor_carrier import MotorCarrier
import time

# Initialize Motor Carrier
mc = MotorCarrier()

print("=" * 60)
print("Battery Enable Test")
print("=" * 60)

# Initialize WITHOUT battery charging
print("\n1. Initializing Motor Carrier (charging disabled)...")
result = mc.begin(enable_charging=False)
if result == 0:
    print("   ✓ Motor Carrier initialized")
else:
    print("   ✗ Initialization failed")

# Check battery reading
print("\n2. Battery reading WITHOUT charging enabled:")
status = mc.battery.get_status()
if status:
    print(f"   Voltage: {status['voltage']:.2f}V")
    print(f"   Raw ADC: {status['raw']}")
    print(f"   Status: {status['status']}")

# Now try enabling battery charging
print("\n3. Enabling battery charging/PMIC...")
try:
    mc.controller.enable_battery_charging()
    print("   ✓ Battery charging enabled")
except Exception as e:
    print(f"   ✗ Error enabling charging: {e}")

# Wait a moment
time.sleep(0.5)

# Check battery reading again
print("\n4. Battery reading WITH charging enabled:")
status = mc.battery.get_status()
if status:
    print(f"   Voltage: {status['voltage']:.2f}V")
    print(f"   Raw ADC: {status['raw']}")
    print(f"   Status: {status['status']}")

    if status['raw'] != 1023:
        print("\n   ✓ Battery reading changed! PMIC enable helped.")
    else:
        print("\n   ⚠️  Battery reading unchanged.")

print("\n" + "=" * 60)
print("Troubleshooting Steps:")
print("=" * 60)
print("1. Check Motor Carrier power switch is ON")
print("2. Verify battery is properly connected")
print("3. Measure battery voltage with multimeter")
print("4. Check battery connector polarity (red=+, black=-)")
print("5. Look for LED indicators on Motor Carrier")
print("=" * 60)
