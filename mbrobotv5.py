# MicroPython library for Maqueen lite v5 based on TigerJython (big shout out to these peeps)
# Original TigerJython libraries are licensed under Mozilla Public License (MPL 2.0)
# Updated 29/08/26

from microbit import i2c, pin1, pin2, pin15, sleep
import machine, music, neopixel

A=0x10
ML=0
MR=2
RGB_L=11
RGB_R=12
SERVO_1=20
SERVO_2=21
LINE_STATE=29
LINE_ADC=(32,34,36)       # left, middle, right
LIGHT=(41,43)             # left, right

# Public position constants
LEFT=0
MIDDLE=1
RIGHT=2

# Motor settings
_speed=50
_offset=0
_diff=0
_arc=0
_powerL=50
_powerR=50

# Motor power curve for speeds 0..100
_PWR=bytes(b'\x00\x0b\x0b\x0c\x0c\x0d\x0d\x0d\x0e\x0e\x0f\x0f\x0f\x10\x10\x11\x11\x11\x12\x12\x13\x13\x13\x14\x14\x15\x15\x15\x16\x16\x17\x17\x17\x18\x19\x1a\x1b\x1b\x1c\x1d\x1e\x1f\x20\x21\x22\x23\x23\x24\x25\x26\x27\x28\x29\x2a\x2b\x2b\x2c\x2d\x2e\x2f\x30\x31\x32\x33\x34\x36\x38\x3a\x3c\x3f\x41\x44\x46\x49\x4c\x4f\x53\x56\x5a\x5e\x62\x67\x6b\x70\x75\x7b\x81\x87\x8d\x94\x9b\xa3\xab\xb4\xbd\xc6\xd0\xdb\xe6\xf2\xff')

_np=neopixel.NeoPixel(pin15,4)
np_rgb_pixels=_np
_alarm=['c5:1','r','c5:1','r:3']
_b1=bytearray(1)
_motor=bytearray(3)
_servo=bytearray(2)


def _write1(reg):
    _b1[0]=reg
    i2c.write(A,_b1)


def _read16(reg):
    _write1(reg)
    b=i2c.read(A,2,repeat=False)
    return (b[0]<<8)|b[1]


def _motorWrite(reg,direction,power):
    _motor[0]=reg
    _motor[1]=direction
    _motor[2]=power
    i2c.write(A,_motor)


def _setMotors(dirL,powerL,dirR,powerR):
    _motorWrite(ML,dirL,powerL)
    _motorWrite(MR,dirR,powerR)


def _power(speed,offset=0):
    return max(0,min(_PWR[int(speed)]+offset,255))


def init():
    # Initialise the V5 controller. Call once after startup.
    i2c.write(A,bytearray([0x46,1]))
    sleep(100)


def connected():
    # Version length is non-zero when the controller responds.
    try:
        _write1(50)
        return i2c.read(A,1)[0]>0
    except:
        return False


def calibrate(offset,differential=0,arcScaling=0):
    global _offset,_diff,_arc
    _offset=max(-10,min(int(offset),50))
    _diff=max(-150,min(int(differential),150))
    _arc=max(-15,min(int(arcScaling),50))
    setSpeed(_speed)


def setSpeed(speed):
    global _speed,_powerL,_powerR
    _speed=max(0,min(int(speed),100))
    p=_power(_speed,_offset)
    boost=round((1-_speed/100)*abs(_diff)) if _speed else 0
    cut=round((_speed/100)*abs(_diff))
    if _diff>0:
        _powerL,_powerR=p-cut,p+boost
    else:
        _powerL,_powerR=p+boost,p-cut
    _powerL=max(0,min(int(_powerL),255))
    _powerR=max(0,min(int(_powerR),255))


def resetSpeed(): setSpeed(50)
def stop(): _setMotors(0,0,0,0)
def forward(): _setMotors(0,_powerL,0,_powerR)
def backward(): _setMotors(1,_powerL,1,_powerR)
def left(): _setMotors(1,_powerL,0,_powerR)
def right(): _setMotors(0,_powerL,1,_powerR)


def _arcPower(radius):
    outer=_speed
    cm=int(radius*100)
    if outer-max(cm+20,40)<=0:
        outer=min(max(cm+40,40),100)
    inner=0
    if cm>=4:
        flat=(100-outer)//2
        inner=(cm*10-35)/(cm*(11+(_arc-4)/10)+90+flat)*outer
    return _power(int(inner)),_power(int(outer))


def rightArc(radius):
    inner,outer=_arcPower(radius)
    _setMotors(0,outer,0,inner)


def leftArc(radius):
    inner,outer=_arcPower(radius)
    _setMotors(0,inner,0,outer)


class Motor:
    def __init__(self,side):
        if side not in (ML,MR):
            raise ValueError('Use 0 for left or 2 for right')
        self.side=side

    def rotate(self,speed):
        s=max(-100,min(int(speed),100))
        _motorWrite(self.side,1 if s<0 else 0,_power(abs(s),_offset) if s else 0)

    def stop(self):
        _motorWrite(self.side,0,0)


# Onboard light sensors. side: 0=left, 1=right.
def readLightIntensity(side):
    if side not in (0,1):
        raise ValueError('Light side must be 0 or 1')
    return _read16(LIGHT[side])


def readLightLeft(): return _read16(LIGHT[0])
def readLightRight(): return _read16(LIGHT[1])
def readLightPair(): return readLightLeft(),readLightRight()


# Digital line states: 0 or 1.
def readLineDigital():
    _write1(LINE_STATE)
    return i2c.read(A,1,repeat=False)[0]


def readLineLeft(): return 1 if readLineDigital()&4 else 0
def readLineMiddle(): return 1 if readLineDigital()&2 else 0
def readLineRight(): return 1 if readLineDigital()&1 else 0


def readLineSensors():
    # One I2C read gives a consistent snapshot of all three sensors.
    v=readLineDigital()
    return (1 if v&4 else 0,1 if v&2 else 0,1 if v&1 else 0)


# Raw analogue line value. position: 0=left, 1=middle, 2=right.
def readLineADC(position):
    if position not in (0,1,2):
        raise ValueError('Line position must be 0, 1 or 2')
    return _read16(LINE_ADC[position])


def readLineAdcLeft(): return _read16(LINE_ADC[0])
def readLineAdcMiddle(): return _read16(LINE_ADC[1])
def readLineAdcRight(): return _read16(LINE_ADC[2])


def setServo(servo,angle):
    if servo in ('S1','P0'):
        reg=SERVO_1
    elif servo in ('S2','P1'):
        reg=SERVO_2
    else:
        raise ValueError("Use 'S1' or 'S2'")
    angle=int(angle)
    if not 0<=angle<=180:
        raise ValueError('Angle must be 0 to 180')
    _servo[0],_servo[1]=reg,angle
    i2c.write(A,_servo)


def getDistance():
    pin1.write_digital(1)
    pin1.write_digital(0)
    p=machine.time_pulse_us(pin2,1,50000)
    if p<=0: return 255
    cm=(p>>6)+(p>>10)+(p>>11)+(p>>12)+1
    return min(cm,500)


class LEDState:
    OFF=0; RED=1; GREEN=2; YELLOW=3
    BLUE=4; PINK=5; CYAN=6; WHITE=7


def setLED(leftState,rightState=None):
    if rightState is None: rightState=leftState
    i2c.write(A,bytearray([RGB_L,leftState,rightState]))


def setLEDLeft(state): i2c.write(A,bytearray([RGB_L,state]))
def setLEDRight(state): i2c.write(A,bytearray([RGB_R,state]))


def fillRGB(r,g,b):
    c=(max(0,min(int(r),255)),max(0,min(int(g),255)),max(0,min(int(b),255)))
    for i in range(4): _np[i]=c
    _np.show()


def setRGB(position,r,g,b):
    if position not in (0,1,2,3):
        raise ValueError('RGB position must be 0 to 3')
    _np[position]=(max(0,min(int(r),255)),max(0,min(int(g),255)),max(0,min(int(b),255)))
    _np.show()


def clearRGB():
    _np.clear()
    _np.show()


def setAlarm(state):
    if state: music.play(_alarm,wait=False,loop=True)
    else: music.stop()


def beep(): music.pitch(440,200,wait=False)

def readBattery(batteryType=BATTERY_ALKALINE):
    ''' Returns battery percentage 0-100
        batteryType:
        BATTERY_ALKALINE = 1
        BATTERY_LITHIUM = 0
    '''
    i2c.write(A, bytearray([BATTERY_SET, batteryType]))
    sleep(50)
    _write1(BATTERY)
    value = i2c.read(A, 1)[0]
    if value > 100:
        value = 100
    return value

def patrolOn():
    i2c.write(A, bytearray([71,1]))

def patrolOff():
    i2c.write(A, bytearray([71,0]))

def patrolSpeed(level):
    i2c.write(A, bytearray([72,level]))
    pin2.set_pull(pin2.NO_PULL)

pin2.set_pull(pin2.NO_PULL)
delay=sleep
motL=Motor(ML)
motR=Motor(MR)
