"""
Battery Monitoring Example
Monitors battery voltage using Motor Carrier.
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

print("\n=== Battery Monitor ===")
print("Monitoring battery voltage (Ctrl+C to stop)...\n")

# Show initial battery status
status = mc.battery.get_status()
if status:
    print(f"Battery Status: {status['status']}")
    print(f"Message: {status['message']}")
    print(f"Voltage: {status['voltage']:.2f}V")
    print(f"Raw ADC: {status['raw']}")
    print()

try:
    while True:
        # Get voltage using proper conversion
        voltage = mc.battery.get_voltage()
        raw = mc.battery.get_raw()

        # Also show firmware-calculated values for comparison
        converted = mc.battery.get_converted()
        filtered = mc.battery.get_filtered()

        # Read temperature (may not be available on all firmware versions)
        temp = mc.controller.get_temperature()

        # Display readings
        if voltage is not None:
            print(f"Voltage: {voltage:5.2f}V | Raw ADC: {raw:4d} | FW Conv: {converted:2d} | FW Filt: {filtered:2d}", end="")
        else:
            print("Battery reading error", end="")

        if temp is not None:
            print(f" | Temp: {temp:.1f}°C", end="")
        else:
            print(" | Temp: N/A", end="")

        # Check for low battery
        if voltage is not None:
            if voltage < 6.0:
                print(" | ⚠️  NO BATTERY", end="")
            elif voltage < 11.0:
                print(" | ⚠️  LOW BATTERY", end="")

        print()  # New line

        time.sleep(1)

except KeyboardInterrupt:
    print("\n\nMonitoring stopped.")
    print("\nNote: If voltage shows ~4V, this typically means:")
    print("  - No battery is connected to the Motor Carrier")
    print("  - Battery is critically discharged (below safe level)")
    print("  - Check battery connections and charge level")
