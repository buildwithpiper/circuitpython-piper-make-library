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
import board
from digitalio import DigitalInOut, Direction, Pull
from adafruit_debouncer import Debouncer
import math
import grove_ultrasonic_ranger
import adafruit_mcp9808
import adafruit_tcs34725
#import adafruit_dotstar

# TODO - Global lives where? Should be inserted by code generator
digital_view = True

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
        elif type == 'Analog':
            self.pin = analogio.AnalogIn(pin)
        self.name = name

    # Report the pin's state for use by the digital view
    #
    def reportPin(self, pinStr):
        global digital_view
        if digital_view:
            if not pinStr:
                self.pin.direction = Direction.INPUT
                self.pin.pull = Pull.UP
                pinStr = str(float(self.pin.value))
            print(chr(17), self.name, "|", pinStr, chr(16), end="")

    # Sets the pin to be an output at the specified logic level
    #
    def setPin(self, pinState):
        self.pin.direction = Direction.OUTPUT
        self.pin.value = pinState
        self.reportPin(str(pinState))

    # Reads the pin by setting it to an input and setting it's pull-up/down and then returning its value
    # (Note that this means you can't use it to detect the state of output pins)
    #
    def checkPin(self, pinPull):
        self.pin.direction = Direction.INPUT
        self.pin.pull = pinPull
        pinValue = self.pin.value
        self.reportPin(str(float(pinValue)))
        return pinValue

    # Same as checkPin except debounced
    #
    def checkPinDebounced(self, pinPull):
        self.pin.direction = Direction.INPUT
        self.pin.pull = pinPull
        self.debounced.update()
        pinValue = self.debounced.value
        self.reportPin(str(float(pinValue)))
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
        pinValue = self.pin.value / 65536
        self.reportPin(str(pinValue))
        return pinValue * 3.3

# This is specific to pins which are attached to an ultrasonic distance sensor
# and we won't allow GPIO operations for now
#
class piperDistanceSensorPin:
    def __init__(self, pin, name):
        self.pin = grove_ultrasonic_ranger.GroveUltrasonicRanger(pin)
        self.name = name

    def readDistanceSensor(self):
        global digital_view
        if digital_view:
            print(chr(17), self.name + "|D", chr(16), end="")
        try:
            d = self.pin.distance
        except RuntimeError as e:
            d = None
            print("Error reading distance sensor")
        return d

# The temperature sensor is attached to the I2C bus which can be shared
#
class piperTemperatureSensor:
    def __init__(self, i2c_bus):
        self.temperature_sensor = adafruit_mcp9808.MCP9808(i2c_bus)

    def readTemperatureSensor(self):
        global digital_view
        if digital_view:
            print(chr(17), "GP20|D", chr(16), end="")
            print(chr(17), "GP21|D", chr(16), end="")
        return self.temperature_sensor.temperature

# The color sensor is attached to the I2C bus which can be shared
#
class piperColorSensor:
    def __init__(self, i2c_bus):
        self.color_sensor = adafruit_tcs34725.TCS34725(i2c_bus)
        self.color_sensor.gain = 60
        self.mult = 60

    def readColorSensor(self):
        global digital_view
        if digital_view:
            print(chr(17), "GP20|D", chr(16), end="")
            print(chr(17), "GP21|D", chr(16), end="")
            
            r,g,b,clear = self.color_sensor.color_raw
            if clear == 0:
                return (0,0,0) +
            return (min(int((r * self.mult/clear) ** 2.3 * 255),255), min(int((g * self.mult / clear) ** 2.5 * 255), 255), min(int((b * self.mult / clear) ** 2.6 * 255), 255))

    def sensorGain(self, val):
        self.mult = val
        self.color_sensor.gain = val

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
# TODO - add digital view support here
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
        if digital_view:
            print(chr(17), self.name, "|", str(pinValue), chr(16), end="")
        return int(self._cubicScaledDeadband((pinValue / 2 ** 15) - 1) * self.outputScale)

################################################################################
# Blocky support functions
#
def isNumber(n):
    if not (type(n) is int or type(n) is float):
        try:
            n = float(n)
        except:
            return 0
    return n

def consoleClear():
    print(chr(16), end="")

def consolePosition(x, y):
    x = int(x); y = int(y)
    if (x != 10 and y != 10):
        print("".join([chr(x) for x in [2, min(255, max(0, x)), min(255, max(0, y))]]), end="")
    elif (x == 10 and y == 10):
        print("".join([chr(x) for x in [2, 9, 9, 4, 6]]), end="")
    elif (x == 10):
        print("".join([chr(x) for x in [2, 9, min(255, max(0, y)), 4]]), end="")
    else:
        print("".join([chr(x) for x in [2, min(255, max(0, x)), 9, 6]]), end="")

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
        c = (abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])) * 20 / 153
    except:
        return 0
    return c

# compares two numbers (int or float) and outputs a value from 0 (very different) to 100 (the same).
def numberCompare(a, b):
    try:
        c = (1 - abs(a-b)/(abs(a) + abs(b))) * 100
    except:
        return 0
    return c

# compares two strings and outputs a value from 0 (very different) to 100 (the same).
def stringCompare(a, b):
    try:
        c = set(list(a))
        d = set(list(b))
        e = c.intersection(d)
        f = (float(len(e)) / (len(c) + len(d) - len(e))) * 100
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
