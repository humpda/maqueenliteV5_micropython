 For Maqueen Lite V5
# mbrobotv5.py
# Original date: 19/05/26
# Updated: 29/08/26

from microbit import i2c, pin1, pin2, pin15, sleep
import gc
import machine
import music
import neopixel

# I2C configuration
I2C_ADDRESS = 0x10
_add_mq = I2C_ADDRESS

# Maqueen Lite V5 registers
MOTOR_LEFT = 0x00
MOTOR_RIGHT = 0x02
RGB_LEFT = 0x0B
RGB_RIGHT = 0x0C
SERVO_1 = 0x14
SERVO_2 = 0x15
BLACK_ADC_STATE = 0x1D
ADC_COLLECT_0 = 0x1E
ADC_COLLECT_1 = 0x20
ADC_COLLECT_2 = 0x22
ADC_COLLECT_3 = 0x24
ADC_COLLECT_4 = 0x26
LIGHT_LEFT_HIGH = 0x29
LIGHT_LEFT_LOW = 0x2A
LIGHT_RIGHT_HIGH = 0x2B
LIGHT_RIGHT_LOW = 0x2C
BATTERY_SET = 0x2D
BATTERY_LEVEL = 0x2E
VERSION_LENGTH = 0x32
VERSION_DATA = 0x33
SYSTEM_INIT = 0x46
LINE_WALKING = 0x47
LINE_SPEED_GRADE = 0x48
CAR_STATE = 0x49
CROSS_DEFAULT = 0x4B
T1_DEFAULT = 0x4C
T2_DEFAULT = 0x4D
T3_DEFAULT = 0x4E

# Motor state
_speedPercent = 50
_powerByteL = 50
_powerByteR = 50
# Combined write: register, L direction, L power, R direction, R power.
_motorState = bytearray(5)
_servoBytes = bytearray(2)
_powerBytesLUT = bytes(b'\x00\x0b\x0b\x0c\x0c\x0d\x0d\x0d\x0e\x0e\x0f\x0f\x0f\x10\x10\x11\x11\x11\x12\x12\x13\x13\x13\x14\x14\x15\x15\x15\x16\x16\x17\x17\x17\x18\x19\x1a\x1b\x1b\x1c\x1d\x1e\x1f\x20\x21\x22\x23\x23\x24\x25\x26\x27\x28\x29\x2a\x2b\x2b\x2c\x2d\x2e\x2f\x30\x31\x32\x33\x34\x36\x38\x3a\x3c\x3f\x41\x44\x46\x49\x4c\x4f\x53\x56\x5a\x5e\x62\x67\x6b\x70\x75\x7b\x81\x87\x8d\x94\x9b\xa3\xab\xb4\xbd\xc6\xd0\xdb\xe6\xf2\xff')

# Calibration data
_powerOffset = 0
_powerDifferential = 0
_arcScaling = 0

# Signalling objects
_underglowNP = neopixel.NeoPixel(pin15, 4)
np_rgb_pixels = _underglowNP
_alarmSequence = ['c5:1', 'r', 'c5:1', 'r:3']

_UNCONNECTEDERRORMSG = "Please connect to Maqueen robot and switch it on."
_buff1 = bytearray(1)
_buff2 = bytearray(2)


def _wr1(reg):
    """Select a one-byte Maqueen register."""
    _buff1[0] = reg
    try:
        i2c.write(_add_mq, _buff1)
    except:
        raise RuntimeError(_UNCONNECTEDERRORMSG)


def _wr2(reg, val):
    """Write a one-byte value to a Maqueen register."""
    _buff2[0] = reg
    _buff2[1] = val
    try:
        i2c.write(_add_mq, _buff2)
    except:
        raise RuntimeError(_UNCONNECTEDERRORMSG)


def _read16be(reg):
    """Select reg and read a two-byte, big-endian unsigned value."""
    _wr1(reg)
    try:
        data = i2c.read(I2C_ADDRESS, 2, repeat=False)
    except:
        raise RuntimeError(_UNCONNECTEDERRORMSG)
    return (data[0] << 8) | data[1]


def _setMotors(dirL, powerL, dirR, powerR):
    """Write both motor states via I2C."""
    _motorState[0] = MOTOR_LEFT
    _motorState[1] = dirL
    _motorState[2] = powerL
    _motorState[3] = dirR
    _motorState[4] = powerR
    try:
        i2c.write(I2C_ADDRESS, _motorState)
    except:
        raise RuntimeError(_UNCONNECTEDERRORMSG)


def _setSingleMotor(side, direction, power):
    """Write one motor. side is 0 for left or 2 for right."""
    if side != MOTOR_LEFT and side != MOTOR_RIGHT:
        raise ValueError("Motor side must be 0 for left or 2 for right.")
    _motorState[0] = MOTOR_LEFT
    _motorState[1 + side] = direction
    _motorState[2 + side] = power
    try:
        i2c.write(I2C_ADDRESS, _motorState)
    except:
        raise RuntimeError(_UNCONNECTEDERRORMSG)


def _getPowerByteLUT(speed, offset):
    """Lookup-table version of the motor power curve."""
    return max(0, min(_powerBytesLUT[speed] + offset, 255))


def _getArcBytes(r):
    """Compute inner and outer motor power for an arc of radius r metres."""
    outerSpeed = _speedPercent
    rCm = int(r * 100)
    threshold = outerSpeed - max(rCm + 20, 40)
    if threshold <= 0:
        outerSpeed = min(max(rCm + 40, 40), 100)
    reducedSpeed = 0
    if rCm >= 4:
        flattening = (100 - outerSpeed) // 2
        reducedSpeed = ((rCm * 10 - 35) /
                        (rCm * (11 + (_arcScaling - 4) / 10) +
                         90 + flattening))
        reducedSpeed = reducedSpeed * outerSpeed
    innerByte = _getPowerByteLUT(int(reducedSpeed), 0)
    outerByte = _getPowerByteLUT(int(outerSpeed), 0)
    return innerByte, outerByte


# Movement functions

def calibrate(offset, differential=0, arcScaling=0):
    """Adjust minimum motor power, L/R differential and arc scaling."""
    global _powerDifferential, _powerOffset, _arcScaling
    _powerOffset = max(min(int(offset), 50), -10)
    _powerDifferential = max(min(int(differential), 150), -150)
    _arcScaling = max(min(int(arcScaling), 50), -15)
    setSpeed(_speedPercent)


def setSpeed(speed):
    """Set the speed used by later movement commands, from 0 to 100%."""
    global _speedPercent, _powerByteL, _powerByteR
    _speedPercent = int(min(max(speed, 0), 100))
    powerByte = _getPowerByteLUT(_speedPercent, _powerOffset)
    boost = (round((1 - _speedPercent / 100) * abs(_powerDifferential))
             if _speedPercent > 0 else 0)
    reduction = round((_speedPercent / 100) * abs(_powerDifferential))
    if _powerDifferential > 0:
        _powerByteL = powerByte - reduction
        _powerByteR = powerByte + boost
    else:
        _powerByteL = powerByte + boost
        _powerByteR = powerByte - reduction
    _powerByteL = int(min(max(_powerByteL, 0), 255))
    _powerByteR = int(min(max(_powerByteR, 0), 255))


def resetSpeed():
    setSpeed(50)


def stop():
    _setMotors(0, 0, 0, 0)


def forward():
    _setMotors(0, _powerByteL, 0, _powerByteR)


def backward():
    _setMotors(1, _powerByteL, 1, _powerByteR)


def left():
    _setMotors(1, _powerByteL, 0, _powerByteR)


def right():
    _setMotors(0, _powerByteL, 1, _powerByteR)


def rightArc(radius):
    inner, outer = _getArcBytes(radius)
    _setMotors(0, outer, 0, inner)


def leftArc(radius):
    inner, outer = _getArcBytes(radius)
    _setMotors(0, inner, 0, outer)


class Motor:
    def __init__(self, side):
        """Create a motor: 0=left, 2=right."""
        if side != MOTOR_LEFT and side != MOTOR_RIGHT:
            raise ValueError("Motor side must be 0 for left or 2 for right.")
        self._side = side

    def rotate(self, speed):
        """Rotate at -100 to 100%. Negative values run backwards."""
        speedClamped = int(min(max(abs(speed), 0), 100))
        power = _getPowerByteLUT(speedClamped, _powerOffset)
        if speed == 0:
            direction = 0
            power = 0
        else:
            direction = 0 if speed > 0 else 1
        _setSingleMotor(self._side, direction, power)

    def stop(self):
        _setSingleMotor(self._side, 0, 0)


# Onboard V5 light sensors
class LightSensor:
    LEFT = 0
    RIGHT = 1


def readLightIntensity(side):
    """Read a V5 light sensor. side: 0=left, 1=right."""
    if side == LightSensor.LEFT:
        return _read16be(LIGHT_LEFT_HIGH)
    if side == LightSensor.RIGHT:
        return _read16be(LIGHT_RIGHT_HIGH)
    raise ValueError("Light sensor side must be 0 for left or 1 for right.")


def readLightLeft():
    return readLightIntensity(LightSensor.LEFT)


def readLightRight():
    return readLightIntensity(LightSensor.RIGHT)


def readLightPair():
    return readLightLeft(), readLightRight()


# Servos

def setServo(servo, angle):
    """Move S1/S2 to an angle from 0 to 180 degrees."""
    if servo == 'S1' or servo == 'P0':
        _servoBytes[0] = SERVO_1
    elif servo == 'S2' or servo == 'P1':
        _servoBytes[0] = SERVO_2
    else:
        raise ValueError("Unknown Servo. Please use 'S1' or 'S2'.")
    if angle < 0 or angle > 180:
        raise ValueError("Invalid angle. Must be between 0 and 180.")
    _servoBytes[1] = int(angle)
    try:
        i2c.write(I2C_ADDRESS, _servoBytes)
    except:
        raise RuntimeError(_UNCONNECTEDERRORMSG)


# Line-tracking sensors
class IRSensor:
    _address = bytes(b'\x1D')

    def __init__(self, index):
        """Create an IR sensor. Firmware bit index must be 0 to 4."""
        if index < 0 or index > 4:
            raise ValueError("IR sensor index must be between 0 and 4.")
        self._index = index

    '''def read_digital(self):
        """Return 0 for dark and 1 for bright."""
        try:
            i2c.write(I2C_ADDRESS, IRSensor._address)
            value = ~i2c.read(I2C_ADDRESS, 1)[0]
        except:
            raise RuntimeError(_UNCONNECTEDERRORMSG)
        return (value & (1 << self._index)) >> self._index
    '''

def readLineDigital():
    _wr1(29) # BLACK_ADC_STATE
    value = i2c.read(0x10, 1)[0]
    return value

def readLineLeft():
    value = readLineDigital()
    return 1 if (value & 0x04) else 0

def readLineMiddle():
    value = readLineDigital()
    return 1 if (value & 0x02) else 0

def readLineRight():
    value = readLineDigital()
    return 1 if (value & 0x01) else 0


class LineSensor:
    LEFT = 0
    MIDDLE = 1
    RIGHT = 2
    


_LINE_ADC_REGISTERS = (
    ADC_COLLECT_0,
    ADC_COLLECT_1,
    ADC_COLLECT_2
)


def readLineADC(position):
    if position < 0 or position > 2:
        raise ValueError("Line sensor position must be between 0 and 2.")
    _wr1(_LINE_ADC_REGISTERS[position])
    buf = i2c.read(0x10, 2, repeat=False)
    return (buf[0] << 8) | buf[1]

def readLineAdcLeft():
    _wr1(32)
    buf = i2c.read(0x10, 2)
    return (buf[0] << 8) | buf[1]


def readLineAdcMiddle():
    _wr1(34)
    buf = i2c.read(0x10, 2)
    return (buf[0] << 8) | buf[1]


def readLineAdcRight():
    _wr1(36)
    buf = i2c.read(0x10, 2)
    return (buf[0] << 8) | buf[1]

def readLineSensors():
    return readLineLeft(), readLineMiddle(), readLineRight()


# Ultrasonic sensor

def getDistance():
    """Read ultrasonic distance in centimetres; returns 255 on timeout."""
    pin1.write_digital(1)
    pin1.write_digital(0)
    pulse = machine.time_pulse_us(pin2, 1, 50000)
    cm = ((pulse >> 6) + (pulse >> 10) + (pulse >> 11) +
          (pulse >> 12) + 1)
    return max(min(cm, 500), 0) if cm > 0 else 255


# Front RGB headlights
class LEDState:
    OFF = 0
    RED = 1
    GREEN = 2
    YELLOW = 3
    BLUE = 4
    PINK = 5
    CYAN = 6
    WHITE = 7


def setLED(state, stateR=None):
    """Set both front RGB headlights."""
    if stateR is None:
        stateR = state
    try:
        i2c.write(I2C_ADDRESS, bytearray([RGB_LEFT, state, stateR]))
    except:
        raise RuntimeError(_UNCONNECTEDERRORMSG)


def setLEDLeft(state):
    try:
        i2c.write(I2C_ADDRESS, bytearray([RGB_LEFT, state]))
    except:
        raise RuntimeError(_UNCONNECTEDERRORMSG)


def setLEDRight(state):
    try:
        i2c.write(I2C_ADDRESS, bytearray([RGB_RIGHT, state]))
    except:
        raise RuntimeError(_UNCONNECTEDERRORMSG)


# Underglow NeoPixels

def fillRGB(red, green, blue):
    """Set all four underside RGB LEDs."""
    red = int(min(max(red, 0), 255))
    green = int(min(max(green, 0), 255))
    blue = int(min(max(blue, 0), 255))
    for i in range(4):
        _underglowNP[i] = (red, green, blue)
    _underglowNP.show()


def clearRGB():
    """Turn off all underside RGB LEDs."""
    _underglowNP.clear()
    _underglowNP.show()


def setRGB(position, red, green, blue):
    """Set one underside RGB LED, position 0 to 3."""
    if position < 0 or position > 3:
        raise ValueError("Invalid RGB-LED position. Must be 0, 1, 2 or 3.")
    red = int(min(max(red, 0), 255))
    green = int(min(max(green, 0), 255))
    blue = int(min(max(blue, 0), 255))
    _underglowNP[position] = (red, green, blue)
    _underglowNP.show()


# Sound

def setAlarm(state):
    """state: 0=off, non-zero=on."""
    if state:
        music.play(_alarmSequence, wait=False, loop=True)
    else:
        music.stop()


def beep():
    music.pitch(440, 200, wait=False)


def collectGarbage():
    gc.collect()


# Default instances and compatibility aliases
pin2.set_pull(pin2.NO_PULL)
delay = sleep

# Original aliases retained. These follow the original library's bit indexes.
irL = IRSensor(0)
irR = IRSensor(2)
IrM = IRSensor(1)

motL = Motor(MOTOR_LEFT)
motR = Motor(MOTOR_RIGHT)
