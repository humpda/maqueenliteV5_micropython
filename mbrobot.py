# mbrobot.py
# Date 10/09/24

from microbit import i2c, pin1, pin2, pin8, pin12, pin13, pin14, pin15, sleep
import gc
import machine
import music
import neopixel

# Motor state
_speedPercent = 50
_powerByteL = 40
_powerByteR = 40
_motorState = bytearray(5)
_servoBytes = bytearray(2)
_powerBytesLUT = bytes(b'\x00\x0b\x0b\x0c\x0c\x0d\x0d\x0d\x0e\x0e\x0f\x0f\x0f\x10\x10\x11\x11\x11\x12\x12\x13\x13\x13\x14\x14\x15\x15\x15\x16\x16\x17\x17\x17\x18\x19\x1a\x1b\x1b\x1c\x1d\x1e\x1f\x20\x21\x22\x23\x23\x24\x25\x26\x27\x28\x29\x2a\x2b\x2b\x2c\x2d\x2e\x2f\x30\x31\x32\x33\x34\x36\x38\x3a\x3c\x3f\x41\x44\x46\x49\x4c\x4f\x53\x56\x5a\x5e\x62\x67\x6b\x70\x75\x7b\x81\x87\x8d\x94\x9b\xa3\xab\xb4\xbd\xc6\xd0\xdb\xe6\xf2\xff')

# Calibration data
_powerOffset = 0
_powerDifferential = 0
_arcScaling = 0

# signaling objects
_underglowNP = neopixel.NeoPixel(pin15, 4)
np_rgb_pixels = _underglowNP
_alarmSequence = ['c5:1', 'r', 'c5:1', 'r:3']

_UNCONNECTEDERRORMSG = "Please connect to Maqueen robot and switch it on."
_buff1 = bytearray(1)
_buff2 = bytearray(2)
# Utility functions

def _wr1(reg):
    _buff1[0] = reg
    i2c.write(_add_mq, _buff1)

def _wr2(reg, val):
    _buff2[0] = reg
    _buff2[1] = val
    i2c.write(_add_mq, _buff2)


def _setMotors(dirL, powerL, dirR, powerR):
    # """Write Motor State via i2c

    # Parameters:
    #     dirL (0/1): Direction of left Wheel. 0=forward, 1=backward
    #     powerL (int): Power of left Wheel in range [0,255].
    #     dirR (0/1): Direction of right Wheel. 0=forward, 1=backward
    #     powerR (int): Power of right Wheel in range [0,255].
    #
    # raises:
    #     RuntimeError: if Robot is switched off or unconnected. (replaces ENODEV error)
    # """
    global _motorState
    _motorState[1] = dirL
    _motorState[2] = powerL
    _motorState[3] = dirR
    _motorState[4] = powerR
    try:
        i2c.write(0x10, _motorState)
    except:
        raise RuntimeError(_UNCONNECTEDERRORMSG)


def _setSingleMotor(side, dir, power):
    # """Write Motor State of a single Motor via i2c

    # Parameters:
    #     side (0/2): Selection of the Wheel. 0=left, 2=right
    #     dir (0/1): Direction for that Wheel. 0=forward, 1=backward
    #     power (int): Power for that Wheel in range [0,255].
    #
    # raises:
    #     RuntimeError: if Robot is switched off or unconnected. (replaces ENODEV error)
    # """
    global _motorState
    _motorState[1 + side] = dir
    _motorState[2 + side] = power
    try:
        i2c.write(0x10, _motorState)
    except:
        raise RuntimeError(_UNCONNECTEDERRORMSG)

# _getPowerByte was used to create LookUpTable (_powerBytesLUT) used in _getPowerByteLUT to reduce Memory Leaks.
# def _getPowerByte(speed, offset):
#     # """Computes the power value for a given speed in %.
#     # It accouts for the nonlinearity of motor strength.
#     # This is the original function that generates the LUT.
#
#     # Parameter:
#     #     speed (number): Desired speed of the Robot in %. Range [0,100].
#     #     offset (number): basic power offset to add to the function.
#     # Returns:
#     #     int: Power in range [0,255] to write to the motors via i2c.
#     # """
#     if speed <= 0:
#         return 0
#     if speed <= 33:
#         return int(0.4 * speed + 11 + offset)
#     elif speed <= 63:
#         return int(0.89 * speed - 5 + offset)
#     elif speed < 100:
#         return int(min(1.0561 ** speed + 20 + offset, 255))
#     else:
#         return 255


def _getPowerByteLUT(speed, offset):
    # """Lookup Table version of _getPowerByte"""
    return min(_powerBytesLUT[speed] + offset, 255)


def _getArcBytes(r):
    # """Computes the power bytes to drive an arc.

    # Parameter:
    #     r (float): Radius in meters of the desired Arc.
    #         Measured from the center of the axle.
    # Returns:
    #     (int, int): Power byte values for each motor.
    #         First the outer Wheels byte [0,255],
    #         then the inner Wheels byte [0,255].
    # """
    outerSpeed = _speedPercent
    rCm = int(r * 100)  # r in cm instead of meters
    # adjust outer speed for unhealthy combinations
    # That is: too low speeds and big arcs.
    threshold = outerSpeed - max(rCm + 20, 40)
    if threshold <= 0:
        outerSpeed = min(max(rCm + 40, 40), 100)
    reducedSpeed = 0
    if rCm >= 4:  # minimal radius is half the axle size: 3.5 cm rounded.
        flattening = (100 - outerSpeed) // 2
        reducedSpeed = (rCm * 10 - 35) / \
            (rCm * (11 + (_arcScaling-4)/10) + 90 + flattening)
        reducedSpeed = reducedSpeed * outerSpeed
    innerByte = _getPowerByteLUT(int(reducedSpeed), 0)
    outerByte = _getPowerByteLUT(int(outerSpeed), 0)
    return (innerByte, outerByte)

# Movement Functions


def calibrate(offset, differential=0, arcScaling=0):
    # """Adjust the driving behaviour of the robots

    # Parameters:
    #     offset (int): Offsets the minimal power of the motors.
    #         Range [-10,50]. Highly affected by battery level.
    #         Adjust this value until it starts moving at speed 1%.
    #     differential (int, optional): Adjusts power difference
    #         of left and right Wheel. Range [-150, 150].
    #         Varies unpredictably with different speeds.
    #         If a Robot steers left when driving forward: negative value
    #         If a Robot steers right when driving forward: positive value
    #         Perfectly straight driving Robots can leave this at 0.
    #     arcScaling (int, optional): Adjusts the radius
    #         driven by leftArc/rightArc. Valid range [-50, 50].
    #         If the Robots radius is too large: positive value
    #         If the Robots radius is too small: negative value
    #         This then adjusts all radii for this Robot, by
    #         scaling it's internal function to the new range.
    # """
    global _powerDifferential
    global _powerOffset
    global _arcScaling
    _powerOffset = max(min(int(offset), 50), -10)
    _powerDifferential = max(min(int(differential), 150), -150)
    _arcScaling = max(min(arcScaling, 50), -15)
    setSpeed(_speedPercent)


def setSpeed(speed):
    # """sets the speed for future motion

    # Parameter:
    #     speed (int): in Range [0,100] as % of desired velocity.
    # """
    global _speedPercent
    global _powerByteL
    global _powerByteR
    _speedPercent = int(min(max(speed, 0), 100))
    powerByte = _getPowerByteLUT(_speedPercent, _powerOffset)
    boost = round((1 - _speedPercent / 100) *
                  abs(_powerDifferential)) if _speedPercent > 0 else 0
    reduction = round((_speedPercent / 100) * abs(_powerDifferential))
    if _powerDifferential > 0:
        _powerByteL = powerByte - reduction
        _powerByteR = powerByte + boost
    else:
        _powerByteL = powerByte + boost
        _powerByteR = powerByte - reduction


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
    # """radius must be given in meters."""
    inner, outer = _getArcBytes(radius)
    _setMotors(0, outer, 0, inner)


def leftArc(radius):
    # """radius must be given in meters."""
    inner, outer = _getArcBytes(radius)
    _setMotors(0, inner, 0, outer)


class Motor:
    def __init__(self, side):
        # """Create a single motor.

        # Parameter:
        #     side (8/2): 0=left, 2=right
        # """
        self._side = side

    def rotate(self, speed):
        # """Controls rotation of this motor.

        # Parameters:
        #     speed (int): Desired speed in %.
        #         Valid range [-100,100].
        #         Negative values are for backward turning.
        # """
        speedClamped = int(min(max(abs(speed), 0), 100))
        power = _getPowerByteLUT(speedClamped, _powerOffset)
        direction = 0 if speed > 0 else 1
        _setSingleMotor(self._side, direction, power)

def readLightIntensity(side):
    _wr1(78)
    LightBuffer = i2c.read(0x10, 4, repeat=False)
    if side == 1:

        return LightBuffer[0] << 8 | LightBuffer[1]
    else:
        return LightBuffer[2] << 8 | LightBuffer[3]




def setServo(servo, angle):
    # """Moves the Servo to position angle.
    # Servos must be connected to the Maqueen Lite's Servo connectors.
    # They are located in front of the left wheel.

    # Parameters:
    #     servo (str): Desired Servo Port. Either 'S1' or 'S2'.
    #     angle (int): Desired angle in degrees. Range [0,180].

    # raises:
    #     ValueError: if arguments are out of valid range
    #     RuntimeError: if Robot is switched off or unconnected.
    # """
    global _servoBytes
    # Acceptance of "P0" and "P1" is for compatibility with V2.
    if servo == 'S1' or servo == 'P0':
        _servoBytes[0] = 0x14
    elif servo == 'S2' or servo == 'P1':
        _servoBytes[0] = 0x15
    else:
        raise ValueError("Unknown Servo. Please use 'S1' or 'S2'.")

    if angle < 0 or angle > 180:
        raise ValueError("Invalid angle. Must be between 0 and 180")

    _servoBytes[1] = angle
    try:
        i2c.write(0x10, _servoBytes)
    except:
        raise RuntimeError(_UNCONNECTEDERRORMSG)

# Sensor functions


class IRSensor():
    def __init__(self, pin):
        # """Create a new IR sensor.

        # Parameter:
        #     pin (object): pin13=left, pin14=right
        # """
        self._pin = pin

    def read_digital(self):
        # """Returns if the surface below is dark or bright.
        # Result can be adjusted by putting the sensor on the dark
        # surface and pressing the LineKey calibration button on the
        # Robot for a few seconds, until the LED's blink.

        # Returns:
        #     0 if the surface is dark. No light was reflected (in Air).
        #     1 if the surface is bright. A lot of light was reflected.
        # """
        return self._pin.read_digital()

    def read_analog(self):
        raise NameError(
            "Maqueen Lite does not support reading analog sensor values.")


def getDistance():
    # """uses the ultrasonic sensor to measure distance

    # Returns:
    #     int: valid Distance as cm in range [0,500].
    #         For measurement errors or larger distances: 255.
    # """
    pin1.write_digital(1)
    pin1.write_digital(0)
    p = machine.time_pulse_us(pin2, 1, 50000)
    # approximate division: cm = p / 57.5
    cm = (p >> 6) + (p >> 10) + (p >> 11) + (p >> 12) + 1
    return max(min(cm, 500), 0) if cm > 0 else 255

# Signaling functions

def setLEDs(rgbl, rgbr):
    _wr2(11, rgbl)
    _wr2(12, rgbr)

def setLED(rgb):
    setLEDs(rgb, rgb)
	
def setLEDLeft(rgbl):
    _wr2(11, rgbl)
	
def setLEDRight(rgbr):
    _wr2(12, rgbr)	

def fillRGB(red, green, blue):
    # """Uses Neopixel to set all 4 bottom RGB LEDs color.
    # Parameters (red,green,blue) are each a byte in Range [0,255].
    # """
    for i in range(4):
        _underglowNP[i] = (red, green, blue)
    _underglowNP.show()


def clearRGB():
    _underglowNP.clear()


def setRGB(position, red, green, blue):
    # """Uses Neopixel to set a single RGB LED of the robot.

    # Parameters:
    #     position (int): position of the targeted LED.
    #         Numbers are visible at underside of Robot.
    #         0=front left
    #         1=back left
    #         2=back right
    #         3=front right
    #     red, green, blue (int): color byte value.
    #         each in range [0,255].

    # raises:
    #     ValueError: if position argument is out of valid range
    # """
    if position < 0 or position > 3:
        raise ValueError("invalid RGB-LED position. Must be 0,1,2 or 3.")
    _underglowNP[position] = (red, green, blue)
    _underglowNP.show()


def setAlarm(state):
    # """state: 0=Off, 1=On"""
    if state:
        music.play(_alarmSequence, wait=False, loop=True)
    else:
        music.stop()


def beep():
    music.pitch(440, 200, wait=False)


# Default instances
pin2.set_pull(pin2.NO_PULL)
delay = sleep
irLeft = IRSensor(pin13)
irRight = IRSensor(pin14)
motL = Motor(0)
motR = Motor(2)
