"""
Debug script to see raw I2C data for battery and temperature readings.
"""

from machine import I2C, Pin
import struct
import time

# I2C setup
i2c = I2C(0, scl=Pin(12), sda=Pin(11), freq=100000)
address = 0x66

# Command codes
GET_RAW_ADC_BATTERY = 0x0D
GET_CONVERTED_ADC_BATTERY = 0x0E
GET_FILTERED_ADC_BATTERY = 0x0F
GET_INTERNAL_TEMP = 0x19

def read_command(command_name, command_code):
    """Read a command and show raw bytes."""
    print(f"\n{command_name} (cmd=0x{command_code:02x}):")
    try:
        # Send command
        i2c.writeto(address, bytes([command_code, 0x00]))
        time.sleep_ms(10)

        # Read 5 bytes (1 IRQ + 4 data)
        response = i2c.readfrom(address, 5)

        print(f"  Raw bytes: {' '.join('%02x' % b for b in response)}")
        print(f"  IRQ status: 0x{response[0]:02x}")

        # Data bytes (skip IRQ)
        data = response[1:]
        print(f"  Data bytes: {' '.join('%02x' % b for b in data)}")

        # Try different interpretations
        # Little-endian signed int
        value_signed = struct.unpack('<i', data)[0]
        print(f"  As signed int (little-endian): {value_signed}")

        # Little-endian unsigned int
        value_unsigned = struct.unpack('<I', data)[0]
        print(f"  As unsigned int (little-endian): {value_unsigned}")

        # Big-endian signed int
        value_be_signed = struct.unpack('>i', data)[0]
        print(f"  As signed int (big-endian): {value_be_signed}")

        # Individual bytes as integers
        print(f"  As bytes: [{data[0]}, {data[1]}, {data[2]}, {data[3]}]")

    except Exception as e:
        print(f"  ERROR: {e}")
        import sys
        sys.print_exception(e)

print("=" * 60)
print("Motor Carrier I2C Debug Readings")
print("=" * 60)

# Battery readings
read_command("Battery RAW", GET_RAW_ADC_BATTERY)
read_command("Battery CONVERTED", GET_CONVERTED_ADC_BATTERY)
read_command("Battery FILTERED", GET_FILTERED_ADC_BATTERY)

# Temperature reading
read_command("Temperature", GET_INTERNAL_TEMP)

print("\n" + "=" * 60)
print("Expected interpretations:")
print("  Battery RAW: Should be ~1000-4000 (ADC value)")
print("  Battery CONVERTED: Should be ~11-13 (voltage)")
print("  Battery FILTERED: Should be ~11-13 (filtered voltage)")
print("  Temperature: Should be ~20000-30000 (millidegrees)")
print("              (divide by 1000 to get degrees C)")
print("=" * 60)
