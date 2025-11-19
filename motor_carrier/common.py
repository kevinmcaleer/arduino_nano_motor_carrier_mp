"""
Common definitions, constants, and I2C commands for Motor Carrier.
Port of Common.h from Arduino library.
"""

# I2C Addresses
MOTOR_CARRIER_ADDRESS = 0x66  # Main Motor Carrier I2C address
PMIC_ADDRESS = 0x6B           # PMIC address for battery charging (Nano 33 IoT)

# I2C Commands
class Command:
    """I2C command codes for Motor Carrier communication."""
    GET_VERSION = 0x01
    RESET = 0x02
    SET_PWM_DUTY_CYCLE_SERVO = 0x03
    SET_PWM_FREQUENCY_SERVO = 0x04
    SET_PWM_DUTY_CYCLE_DC_MOTOR = 0x05
    SET_PWM_FREQUENCY_DC_MOTOR = 0x06
    GET_RAW_COUNT_ENCODER = 0x07
    RESET_COUNT_ENCODER = 0x08
    GET_OVERFLOW_UNDERFLOW_STATUS = 0x09
    GET_COUNT_PER_SECOND_ENCODER = 0x0A
    SET_INTERRUPT_ON_COUNT_ENCODER = 0x0B
    SET_INTERRUPT_ON_VELOCITY_ENCODER = 0x0C
    GET_RAW_ADC_BATTERY = 0x0D
    GET_CONVERTED_ADC_BATTERY = 0x0E
    GET_FILTERED_ADC_BATTERY = 0x0F
    SET_PID_GAIN_CL_MOTOR = 0x10
    RESET_PID_GAIN_CL_MOTOR = 0x11
    SET_CONTROL_MODE_CL_MOTOR = 0x12
    SET_POSITION_SETPOINT_CL_MOTOR = 0x13
    SET_VELOCITY_SETPOINT_CL_MOTOR = 0x14
    SET_MAX_ACCELERATION_CL_MOTOR = 0x15
    SET_MAX_VELOCITY_CL_MOTOR = 0x16
    SET_MIN_MAX_DUTY_CYCLE_CL_MOTOR = 0x17
    PING = 0x18
    GET_INTERNAL_TEMP = 0x19
    CLEAR_IRQ = 0x1A
    GET_FREE_RAM = 0x1B
    GET_PID_VAL = 0x1C

# IRQ Status Values
IRQ_ENCODER_COUNTER_REACHED = 1
IRQ_ENCODER_VELOCITY_REACHED = 2

# Control Modes for PID
class ControlMode:
    """Control modes for closed-loop motor control."""
    OPEN_LOOP = 0
    VELOCITY = 1
    POSITION = 2

# Target Types for PID Setpoint
class TargetType:
    """Target types for PID control."""
    TARGET_VELOCITY = 0
    TARGET_POSITION = 1

# Pin Definitions (Nano 33 IoT compatible defaults)
# Note: These can be customized when creating MotorCarrier object
class Pins:
    """Pin definitions for GPIO control."""
    # IRQ pin from Motor Carrier
    IRQ_PIN = 6

    # M3 Motor pins (direct GPIO control) - Nano 33 IoT defaults
    M3_IN1 = 2
    M3_IN2 = 3

    # M4 Motor pins (direct GPIO control) - Nano 33 IoT defaults
    M4_IN1 = 5
    M4_IN2 = 4

    # I2C pins (can be customized)
    I2C_SCL = 12
    I2C_SDA = 11

# Fixed-Point Math Constants
FIXED_POINT_FRACTIONAL_BITS = 8
FIXED_POINT_SCALE = 1 << FIXED_POINT_FRACTIONAL_BITS  # 256

def float_to_fixed(value):
    """Convert float to Q24.8 fixed-point format (32-bit with 8 fractional bits)."""
    return int(value * FIXED_POINT_SCALE)

def fixed_to_float(value):
    """Convert Q24.8 fixed-point format to float."""
    return value / FIXED_POINT_SCALE

def fixed_to_bytes(value):
    """Convert fixed-point value to 4-byte array (little-endian)."""
    fixed_int = float_to_fixed(value) if isinstance(value, float) else value
    # Handle signed integers for MicroPython compatibility
    if fixed_int < 0:
        fixed_int = (1 << 32) + fixed_int  # Convert to unsigned
    return fixed_int.to_bytes(4, 'little')

def bytes_to_fixed(data):
    """Convert 4-byte array to fixed-point value."""
    value = int.from_bytes(data, 'little')
    # Convert from unsigned to signed if necessary
    if value >= (1 << 31):  # If sign bit is set
        value -= (1 << 32)
    return value
