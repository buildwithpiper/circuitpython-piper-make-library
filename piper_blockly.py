################################################################################
# The MIT License (MIT)
#
# Author: Matthew Matz
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
################################################################################

from digitalio import DigitalInOut, Direction, Pull
# from analogio import AnalogIn    # PI SBC - REMOVED
from adafruit_debouncer import Debouncer
import math
import grove_ultrasonic_ranger
import adafruit_mcp9808
import adafruit_tcs34725
import adafruit_mpu6050
#import adafruit_dotstar
import pwmio
from adafruit_motor import servo
# from gamepadshift import GamePadShift    # PI SBC - TEMPORARY REMOVE - REPLACE WITH keypad
from micropython import const
# from touchio import TouchIn    # PI SBC - REMOVED
import neopixel
from rainbowio import colorwheel
import time

digital_view = True

piper_pin_states = []
piper_pin_names = []

def set_digital_view(state):
    global digital_view
    digital_view = state

################################################################################
# This class is for digital GPIO pins
#
class piperPin:
    def __init__(self, pin, name, type='Digital'):
        if type == 'Digital':
            self.pin = DigitalInOut(pin)
            self.debounced = Debouncer(self.pin)
            self.debounceRose = Debouncer(self.pin)
            self.debounceFell = Debouncer(self.pin)
        #elif type == 'Analog':    # PI SBC - REMOVED
            #self.pin = AnalogIn(pin)    # PI SBC - REMOVED
        self.name = name

    # Report the pin's state for use by the digital view
    #
    def reportPin(self, pinStr):
        global digital_view
        if (digital_view == True):
            if not pinStr:
                self.pin.direction = Direction.INPUT
                self.pin.pull = Pull.UP
                pinStr = float(self.pin.value)
            send_piper_pin_state(self.name, pinStr)
            #print(chr(17), self.name, "|", pinStr, chr(16), end="")

    # Sets the pin to be an output at the specified logic level
    #
    def setPin(self, pinState):
        self.pin.direction = Direction.OUTPUT
        self.pin.value = pinState
        self.reportPin(pinState)

    # Reads the pin by setting it to an input and setting it's pull-up/down and then returning its value
    # (Note that this means you can't use it to detect the state of output pins)
    #
    def checkPin(self, pinPull):
        self.pin.direction = Direction.INPUT
        self.pin.pull = pinPull
        pinValue = self.pin.value
        self.reportPin(float(pinValue))
        return pinValue

    # Same as checkPin except debounced
    #
    def checkPinDebounced(self, pinPull):
        self.pin.direction = Direction.INPUT
        self.pin.pull = pinPull
        self.debounced.update()
        pinValue = self.debounced.value
        self.reportPin(float(pinValue))
        return pinValue

    # Look for rising edge. Typically happens when a button (with pullup)
    # is released.
    #
    def checkPinRose(self, pinPull):
        self.pin.direction = Direction.INPUT
        self.pin.pull = pinPull
        self.debounceRose.update()
        pinValue = self.debounceRose.rose
        self.reportPin(None)
        return pinValue

    # Look for falling edge. Typically happens when a button (with pullup)
    # is pressed.
    #
    def checkPinFell(self, pinPull):
        self.pin.direction = Direction.INPUT
        self.pin.pull = pinPull
        self.debounceFell.update()
        pinValue = self.debounceFell.fell
        self.reportPin(None)
        return pinValue

    # Reads an analog voltage from the specified pin
    #
    def readVoltage(self):
        # pinValue = self.pin.value / 65536    # PI SBC - REMOVED
        # self.reportPin(pinValue)    # PI SBC - REMOVED
        # return pinValue * 3.3
        pinValue = self.pin.value
        self.reportPin(float(pinValue))
        return pinValue * 3.3        

# This is specific to pins which are attached to a servo
# and we won't allow GPIO operations for now
#
class piperServoPin:
    def __init__(self, pin, name):
        # create a PWMOut object on the control pin.
        self.pwm = pwmio.PWMOut(pin, duty_cycle=0, frequency=50)
        self.pin = servo.Servo(self.pwm, min_pulse=580, max_pulse=2350)
        self.name = name

    def setServoAngle(self, a):
        #global digital_view
        #if (digital_view == True):
        #    print(chr(17), self.name + "|D", chr(16), end="")
        send_piper_pin_state(self.name, "P")
        try:
            if a == None:
                self.pin.fraction = None
            else:
                self.pin.angle = a
        except RuntimeError as e:
            print("Error setting servo angle", str(e))

    def setServoFraction(self, f):
        #global digital_view
        #if (digital_view == True):
        #    print(chr(17), self.name + "|D", chr(16), end="")
        send_piper_pin_state(self.name, "P")
        try:
            self.pin.fraction = f
        except RuntimeError as e:
            print("Error setting servo position", str(e))

# This is specific to pins which are used for capacitive sensing sensor
# and we won't allow GPIO operations for now
#
"""   # PI SBC - REMOVED
class piperCapSensePin:
    def __init__(self, pin, name):
        self.pin = TouchIn(pin)
        self.name = name

    def readCapSenseValue(self):
        #global digital_view
        #if (digital_view == True):
        #    print(chr(17), self.name + "|D", chr(16), end="")
        send_piper_pin_state(self.name, "P")
        try:
            d = self.pin.raw_value
        except RuntimeError as e:
            d = None
            print("Error reading capactive sense value", str(e))
        return d
"""

# This is specific to pins which are attached to an ultrasonic distance sensor
# and we won't allow GPIO operations for now
#
class piperDistanceSensorPin:
    def __init__(self, pin, name):
        self.pin = grove_ultrasonic_ranger.GroveUltrasonicRanger(pin)
        self.name = name

    def readDistanceSensor(self):
        #global digital_view
        #if (digital_view == True):
        #    print(chr(17), self.name + "|D", chr(16), end="")
        send_piper_pin_state(self.name, "P")
        try:
            d = self.pin.distance
        except RuntimeError as e:
            d = None
            print("Error reading distance sensor", str(e))
        return d

# The temperature sensor is attached to the I2C bus which can be shared
#
class piperTemperatureSensor:
    def __init__(self, i2c_bus):
        self.temperature_sensor = adafruit_mcp9808.MCP9808(i2c_bus)

    def readTemperatureSensor(self):
        #global digital_view
        #if (digital_view == True):
        #    print(chr(17), "GP20|D", chr(16), end="")
        #    print(chr(17), "GP21|D", chr(16), end="")
        send_piper_pin_state("GP20", "P")
        send_piper_pin_state("GP21", "P")
        return self.temperature_sensor.temperature

# The color sensor is attached to the I2C bus which can be shared
#
class piperColorSensor:
    def __init__(self, i2c_bus):
        self.color_sensor = adafruit_tcs34725.TCS34725(i2c_bus)
        self.color_sensor.gain = 60
        self.mult = pow((128/60), 0.6)

    def readColorSensor(self):
        #global digital_view
        #if (digital_view == True):
        #    print(chr(17), "GP20|D", chr(16), end="")
        #    print(chr(17), "GP21|D", chr(16), end="")
        send_piper_pin_state("GP20", "P")
        send_piper_pin_state("GP21", "P")

        r, g, b, clear = self.color_sensor.color_raw
        if clear == 0:
            return (0, 0, 0)
        
        s = (r ** 1.95 + g ** 2.025 + b * b) / 3
        c1 = clear ** 0.9
        r1 = int(min(r * r * c1 * self.mult / s, 255))
        g1 = int(min(g * g * c1 * self.mult / s, 255))
        b1 = int(min(b * b * c1 * self.mult / s, 255))
        
        return (r1, g1, b1)

    def sensorGain(self, val):
        self.mult = pow((128/val), 0.6)
        self.color_sensor.gain = val

    def read(self):
        return self.readColorSensor()
        
    def sensorGain(self, val):
        self.mult = pow((128/val), 0.6)
        self.color_sensor.gain = val


# The MPU6050 IMU is attached to the I2C bus which can be shared
#
class piperMotionSensor:
    def __init__(self, i2c_bus, address=0x69):
        self.motion_sensor = adafruit_mpu6050.MPU6050(i2c_bus, address=address)

    def readMotionSensor(self):
        #global digital_view
        #if (digital_view == True):
        #    print(chr(17), "GP20|D", chr(16), end="")
        #    print(chr(17), "GP21|D", chr(16), end="")
        send_piper_pin_state("GP20", "P")
        send_piper_pin_state("GP21", "P")

        self.acc_x, self.acc_y, self.acc_z = self.motion_sensor.acceleration
        self.gyro_x, self.gyro_y, self.gyro_z = self.motion_sensor.gyro
        self.temp = self.motion_sensor.temperature
        self.roll = math.atan2(self.acc_y, self.acc_z) * 180 / math.pi
        self.pitch = math.atan2(self.acc_z, self.acc_x) * 180 / math.pi
        self.yaw = math.atan2(self.acc_x, self.acc_y) * 180 / math.pi

    def read(self):
        self.readMotionSensor()

# The Heart sensor is attached to the I2C bus which can be shared
#
class piperHeartSensor:
    def __init__(self, i2c_bus):
        self.heart_sensor = piper_heart_sensor(i2c_bus, 4, 175)  # Use options that are generally effective
        self.raw_value = 0
        self.heart_rate = -1

    def readHeartSensor(self):
        #global digital_view
        #if (digital_view == True):
        #    print(chr(17), "GP20|D", chr(16), end="")
        #    print(chr(17), "GP21|D", chr(16), end="")
        send_piper_pin_state("GP20", "P")
        send_piper_pin_state("GP21", "P")

        self.raw_value = self.heart_sensor.read_sensor()
        if (self.heart_sensor.heart_rate == None):
            self.heart_rate = -1
        else:
            self.heart_rate = self.heart_sensor.heart_rate

    def read(self):
        self.readHeartSensor()

# NeoPixels can be attached to any GPIO pin
#
class piperNeoPixels:
    def __init__(self, pin, name, pixel_count):
        self.pixels = neopixel.NeoPixel(pin, pixel_count, brightness=0.6, auto_write=False)
        self.pin = pin
        self.name = name

    def fill(self, color):
        self.pixels.fill(color)

    def show(self):
        send_piper_pin_state(self.name, "P")
        self.pixels.show()


"""   # PI SBC - REMOVED
# constants associated with the Piper Make Controller
BUTTON_1 = const(128)
BUTTON_2 = const(64)
BUTTON_3 = const(32)
BUTTON_4 = const(16)

BUTTON_5 = const(8)
BUTTON_6 = const(4)
BUTTON_7 = const(2)
BUTTON_8 = const(1)
BUTTON_9 = const(32768)
BUTTON_10 = const(16384)

BUTTON_11 = const(8192)
BUTTON_12 = const(4096)
BUTTON_13 = const(2048)
BUTTON_14 = const(1024)

ANY_BUTTON = const(64767)  # All of the bits used by the controller

# This is specific to pins which are attached to the Piper Make Controller
# and we won't allow GPIO operations for now
#
class piperControllerPins:
    def __init__(self, clock_pin, clock_name, data_pin, data_name, latch_pin, latch_name):
        self.clock_pin = DigitalInOut(clock_pin)
        self.data_pin = DigitalInOut(data_pin)
        self.latch_pin = DigitalInOut(latch_pin)
        
        self.clock_name = clock_name
        self.data_name = data_name
        self.latch_name = latch_name
        
        self.gamepad = GamePadShift(self.clock_pin, self.data_pin, self.latch_pin, 16)

    def readButtons(self):
        #global digital_view
        #if (digital_view == True):
        #    print(chr(17), self.clock_name + "|D", chr(16), end="")
        #    print(chr(17), self.data_name + "|D", chr(16), end="")
        #    print(chr(17), self.latch_name + "|D", chr(16), end="")
        send_piper_pin_state(self.clock_name, "P")
        send_piper_pin_state(self.data_name, "P")
        send_piper_pin_state(self.latch_name, "P")
        try:
            self.buttons = self.gamepad.get_pressed()
        except RuntimeError as e:
            print("Error reading controller buttons", str(e))
        return self.buttons

    def wasPressed(self, b):
        if (self.buttons & b):
            return True
        else:
            return False
"""


# The DotStar is connected to fixed PCB pins
#
#class piperDotStar:
#    def __init__(self):
#        self.dotstar_led = adafruit_dotstar.DotStar(board.APA102_SCK, board.APA102_MOSI, 1)
#        self.dotstar_led.brightness = 0.6
#
#    def setDotStar(self, color):
#        global digital_view
#        self.dotstar_led[0] = color
#        if (digital_view == True):
#            print(chr(17), "DS|", str(color), chr(16), end="")

################################################################################
# This function allows a user to manage joystick handling themselves.
# See http://www.mimirgames.com/articles/games/joystick-input-and-using-deadbands/
# for the motivation and theory
#
class piperJoystickAxis:
    def __init__(self, pin, name, outputScale=20.0, deadbandCutoff=0.1, weight=0.2):
        self.name = name
        self.pin = AnalogIn(pin)
        self.outputScale = outputScale
        self.deadbandCutoff = deadbandCutoff
        self.weight = weight
        self.alpha = self._Cubic(self.deadbandCutoff)

    # Cubic function to map input to output in such a way as to give more precision
    # for lower values
    def _Cubic(self, x):
        return self.weight * x ** 3 + (1.0 - self.weight) * x

    # Eliminate the jump present in the deadband, but use the cubic function to give
    # more precision to lower values
    #
    def _cubicScaledDeadband(self, x):
        if abs(x) < self.deadbandCutoff:
            return 0
        else:
            return (self._Cubic(x) - (copysign(1,x)) * self.alpha) / (1.0 - self.alpha)

    # The analog joystick output is an unsigned number 0 to 2^16, which we
    # will scale to -1 to +1 for compatibility with the cubic scaled
    # deadband article. This will then remap and return a value
    # still in the range -1 to +1. Finally we multiply by the requested scaler
    # an return an integer which can be used with the mouse HID.
    #
    def readJoystickAxis(self):
        pinValue = self.pin.value
        #if (digital_view == True):
        #    print(chr(17), self.name, "|", str(pinValue), chr(16), end="")
        send_piper_pin_state(self.name, pinValue)
        return int(self._cubicScaledDeadband((pinValue / 2 ** 15) - 1) * self.outputScale)

################################################################################
# Blocky support functions
################################################################################

# used to ensure that a value is a number
def isNumber(n):
    if not (type(n) is int or type(n) is float):
        try:
            n = float(n)
        except:
            return 0
    return n

# used to clear the console
def consoleClear():
    print(chr(16), end="")

# used to position the cursor in the console
def consolePosition(x, y):
    x = (min(max(int(x), 0), 255))
    y = (min(max(int(y), 0), 255))
    print(chr(17), 'P', str(x) + ',' + str(y), chr(17), end='')

# used internally to send the state of the pins for the digital view - only sends when the state has changed.
def send_piper_pin_state(_pin_name, _pin_state):
    global piper_pin_states, piper_pin_names
    _current_pin_index = 0
    if (digital_view == True):
        try:
            _current_pin_index = piper_pin_names.index(_pin_name)
        except:
            piper_pin_names.append(_pin_name)
            piper_pin_states.append(-1)
            _current_pin_index = len(piper_pin_names) - 1
        if (_pin_state == 'P'):
            if (time.monotonic() > piper_pin_states[_current_pin_index]):
                print(chr(17), _pin_name + "|D", chr(16), end="")
                piper_pin_states[_current_pin_index] = time.monotonic() + 0.6
        elif (piper_pin_states[_current_pin_index] != _pin_state):
            print(chr(17), _pin_name, "|", str(_pin_state), chr(16), end="")
            piper_pin_states[_current_pin_index] = _pin_state

# instructs the connected computer to play a sound by sending control characters and the name
# (or instructions related to) the specified sound
def playSound(soundName): 
    print(chr(19), soundName, chr(19), end="")

# instructs the connected to computer to display the specified string in the specified color in pop-up
def shout(color, text):
    print(chr(18), str(color) + "|" + str(text), chr(18), end="")

# translates emojis to their corresponding control characters
def emojiCharacter(c):
    if c == "in-love":
        return chr(20)
    if c == "sad":
        return chr(21)
    if c == "happy":
        return chr(22)
    if c == "thinking":
        return chr(23)
    if c == "quiet":
        return chr(24)
    if c == "confused":
        return chr(25)
    if c == "suspicious":
        return chr(26)
    if c == "unhappy":
        return chr(27)
    if c == "bored":
        return chr(28)
    if c == "surprised":
        return chr(29)

# compares two colors (3-tuples) and outputs a value from 0 (opposite) to 100 (the same).
def colorCompare(a, b):
    try:
        c = 100 - int((abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])) * 20 / 153)
    except:
        return 0
    return c

# compares two numbers (int or float) and outputs a value from 0 (very different) to 100 (the same).
def numberCompare(a, b):
    try:
        c = int((1 - abs(a-b)/(abs(a) + abs(b))) * 100)
    except:
        return 0
    return c

# compares two strings and outputs a value from 0 (very different) to 100 (the same).
def stringCompare(a, b):
    try:
        c = set(list(a))
        d = set(list(b))
        e = c.intersection(d)
        f = int((float(len(e)) / (len(c) + len(d) - len(e))) * 100)
    except:
        return 0
    return f

# map (scale) a value from one range (a, b) to a new range (c, d)
def mapValue(value, a, b, c, d):
    return c + ((float(value - a) / float(b - a)) * (d - c))

# helper function for graphing number values
def piperGraphNumbers(graph_values):
    print(chr(17), 'G', ','.join(graph_values), chr(17), end='')
    
# helper function for graphing a color value (tuple)
def piperGraphColor(color_value):
    print(chr(17), 'C', str(color_value), chr(17), end='')

# used to generate an RGB color value form a single integer 0-255
def piperColorWheel(hue_value, bright_value=100):
    if (bright_value < 3):
        return 0
    if (bright_value >= 100):
        return colorwheel(int(hue_value) & 255)
    bright_value /= 100.0
    hue_value = colorwheel(int(hue_value) & 255)
    bv_a = int(((hue_value) & 255) * bright_value) & 255
    bv_b = int(((hue_value >> 8) & 255) * bright_value) & 255
    bv_c = int(((hue_value >> 16) & 255) * bright_value) & 255
    return (bv_a, bv_b, bv_c)
