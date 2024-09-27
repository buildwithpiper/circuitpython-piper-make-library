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
from analogio import AnalogIn
import math
import piper_range_finder
import piper_heart_sensor
import piper_motor_module
from piper_lightshow import Lightshow, PixBuffer
import adafruit_mcp9808
import adafruit_tcs34725
import adafruit_mpu6050
import pwmio
from adafruit_motor import servo
from micropython import const
from touchio import TouchIn
import neopixel
from rainbowio import colorwheel
import time
import random

digital_view = True

piper_pin_states = []
piper_pin_names = []

def set_digital_view(state):
    global digital_view
    digital_view = state

################################################################################
# This class is for digital GPIO pins
class piperPin:
    def __init__(self, pin, name, type='Digital'):
        if type == 'Digital':
            self.pin = DigitalInOut(pin)
        elif type == 'Analog':
            self.pin = AnalogIn(pin)
        elif type == 'Voltage':
            self.pin = pwmio.PWMOut(pin, frequency=5000, duty_cycle=0)
        self.name = name
        self.type = type

    # Report the pin's state for use by the digital view
    def reportPin(self, pinStr):
        global digital_view
        if (digital_view == True):
            if not pinStr:
                self.pin.direction = Direction.INPUT
                self.pin.pull = Pull.UP
                pinStr = float(self.pin.value)
            send_dv_state(self.name, pinStr)

    # Sets the pin to be an output at the specified logic level
    def setPin(self, pinState):
        if (self.type == 'Voltage'):
            # Set the pin state by setting a PWM duty cycle.
            pinValue = max(min(float(pinState), 3.3), 0) / 3.3
            self.pin.duty_cycle = int(65535 * pinValue)
            self.reportPin(pinValue)
        else:
            self.pin.direction = Direction.OUTPUT
            self.pin.value = pinState
            self.reportPin(pinState)

    # Reads the pin by setting it to an input and setting it's pull-up/down and then returning its value
    # (Note that this means you can't use it to detect the state of output pins)
    def checkPin(self, pinPull):
        self.pin.direction = Direction.INPUT
        self.pin.pull = pinPull
        pinValue = self.pin.value
        self.reportPin(float(pinValue))
        return pinValue

    # Reads an analog voltage from the specified pin
    def readVoltage(self):
        pinValue = self.pin.value / 65536
        self.reportPin(pinValue)
        return pinValue * 3.3


# This is specific to pins which are attached to a servo
class piperServoPin:
    def __init__(self, pin, name):
        # create a PWMOut object on the control pin.
        self.pwm = pwmio.PWMOut(pin, duty_cycle=0, frequency=50)
        self.pin = servo.Servo(self.pwm, min_pulse=580, max_pulse=2350)
        self.name = name

    def setServoAngle(self, a):
        send_dv_state(self.name, "P")
        try:
            if a == None:
                self.pin.fraction = None
            else:
                self.pin.angle = a
        except RuntimeError as e:
            print("Error setting servo angle", str(e))

    def setServoFraction(self, f):
        send_dv_state(self.name, "P")
        try:
            self.pin.fraction = f
        except RuntimeError as e:
            print("Error setting servo position", str(e))


# This is specific to pins which are used for capacitive sensing
class piperCapSensePin:
    def __init__(self, pin, name):
        self.pin = TouchIn(pin)
        self.name = name

    def readCapSenseValue(self):
        send_dv_state(self.name, "P")
        try:
            d = self.pin.raw_value
        except RuntimeError as e:
            d = None
            print("Error reading capactive sense value", str(e))
        return d


# This is specific to pins which are attached to an ultrasonic distance sensor
class piperDistanceSensorPin:
    def __init__(self, pin, name):
        self.pin = piper_range_finder.PiperUltrasonicRanger(pin)
        self.name = name

    def readDistanceSensor(self):
        send_dv_state(self.name, "P")
        try:
            d = self.pin.distance
        except RuntimeError as e:
            d = None
            print("Error reading distance sensor", str(e))
        return d


# The temperature sensor is attached to the I2C bus which can be shared
class piperTemperatureSensor:
    def __init__(self, i2c_bus):
        self.temperature_sensor = adafruit_mcp9808.MCP9808(i2c_bus)

    def readTemperatureSensor(self):
        send_dv_i2c()
        return self.temperature_sensor.temperature


# The color sensor is attached to the I2C bus which can be shared
class piperColorSensor:
    def __init__(self, i2c_bus):
        self.color_sensor = adafruit_tcs34725.TCS34725(i2c_bus)
        self.color_sensor.gain = 60
        self.mult = pow((128/60), 0.6)

    def readColorSensor(self):
        send_dv_i2c()
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


# The MPU6050 IMU is attached to the I2C bus which can be shared
class piperMotionSensor:
    def __init__(self, i2c_bus, address=0x69):
        self.motion_sensor = adafruit_mpu6050.MPU6050(i2c_bus, address=address)

    def readMotionSensor(self):
        send_dv_i2c()
        self.acc_x, self.acc_y, self.acc_z = self.motion_sensor.acceleration
        self.gyro_x, self.gyro_y, self.gyro_z = self.motion_sensor.gyro
        self.temp = self.motion_sensor.temperature
        self.roll = math.atan2(self.acc_y, self.acc_z) * 180 / math.pi
        self.pitch = math.atan2(self.acc_z, self.acc_x) * 180 / math.pi
        self.yaw = math.atan2(self.acc_x, self.acc_y) * 180 / math.pi

    def read(self):
        self.readMotionSensor()


# The Heart sensor is attached to the I2C bus which can be shared
class piperHeartSensor:
    def __init__(self, i2c_bus):
        self.heart_sensor = piper_heart_sensor(i2c_bus, smoothing=4, channel=0)  # Use options that are generally effective
        self.raw_value = 0
        self.heart_rate = -1

    def readHeartSensor(self):
        send_dv_i2c()
        self.raw_value = self.heart_sensor.read_sensor()
        if (self.heart_sensor.heart_rate == None):
            self.heart_rate = -1
        else:
            self.heart_rate = self.heart_sensor.heart_rate

    def read(self):
        self.readHeartSensor()

# NeoPixels can be attached to any GPIO pin
class piperNeoPixels:
    def __init__(self, pin, name, pixel_count):
        self.pixels = neopixel.NeoPixel(pin, pixel_count, brightness=0.6, auto_write=False)
        self.pin = pin
        self.name = name

    def fill(self, color):
        self.pixels.fill(color)

    def show(self):
        send_dv_state(self.name, "P")
        self.pixels.show()


# The Lightshow module is connected to SPI0 (17, 18, 19), and the pins are hardcoded
# because they are driven continuously by the second processor on the RP2040
class piperLightshow:
    def __init__(self):
        self.lightshow = Lightshow()
        self.pix = PixBuffer()

    def from_text(self, string, color=63, bgcolor=0, colors=None):
        return self.pix.from_text(string, color, bgcolor, colors)

    def from_grid(self, lines):
        return self.pix.from_grid(lines)

    def pixel(self, x, y, color=None):
        __pixel = self.pix.pixel(x, y, color)
        if __pixel is not None:
            return __pixel

    def box(self, color, x=0, y=0, width=None, height=None):
        self.pix.box(color, x, y, width, height)

    def clear(self):
        self.pix.box(0, 0, 0, 8, 8)

    def draw(self, source, dx=0, dy=0, x=0, y=0, width=None, height=None, key=None):
        self.pix.draw(source, dx, dy, x, y, width, height, key)

    def send_dv_lightshow(self, pixel_array):
        global digital_view
        if (digital_view == True):
            __base_str = 'ABDEFHIJKMNOQSTUVWXYZ{}[]=+:0123456789abcdefghijklmnopqrstuvwxyz'
            print(chr(17), 'L', ''.join([__base_str[__i] for __i in list(pixel_array)]), chr(17))
        
    def show(self):
        self.lightshow.show(self.pix)
        self.send_dv_lightshow(self.pix.buffer)

    def format_color(self, color):
        return self.lightshow.rgb_to_byte(color)

# The Motor Module is attached to the I2C bus which can be shared
class piperMotorModule:
    def __init__(self, i2c_bus):
        self.motor_module = piper_motor_module(i2c_bus)

    # set the specificed motor to coast
    def coast(self, motor=0):
        send_dv_i2c()
        self.motor_module.coast(motor)

    # set the specificed motor to brake
    def brake(self, motor=0):
        send_dv_i2c()
        self.motor_module.brake(motor)

    # stop the motors (coast)
    def stop(self):
        send_dv_i2c()
        self.motor_module.stop()

    # set the specificed motor to coast
    def set_speed(self, motor=0, speed=0):
        send_dv_i2c()
        self.motor_module.set_speed(motor, speed)

    # set the angle of the servo
    def servo_angle(self, servo=0, angle=0):
        send_dv_i2c()
        self.motor_module.servo_angle(servo, angle)

    # detach the servo
    def servo_stop(self, servo=0):
        send_dv_i2c()
        self.motor_module.servo_stop(servo)


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

BUTTON_15 = const(512)
BUTTON_16 = const(256)

ANY_BUTTON = const(64767)  # All of the bits used by the 14 button controller
ANY_BUTTON_16 = const(65535)  # All of the bits used by the 16 button controller

# This is specific to pins which are attached to the Piper Make Controller
class piperControllerPins:
    def __init__(self, clock_pin, clock_name, data_pin, data_name, latch_pin, latch_name):
        self.clock_pin = DigitalInOut(clock_pin)
        self.data_pin = DigitalInOut(data_pin)
        self.latch_pin = DigitalInOut(latch_pin)
        
        self.clock_name = clock_name
        self.data_name = data_name
        self.latch_name = latch_name
        self.bit_count = 16

        self.last = int('0' * self.bit_count, 2)
        self.pressed = self.last

    def readButtons(self):
        send_dv_state(self.clock_name, "P")
        send_dv_state(self.data_name, "P")
        send_dv_state(self.latch_name, "P")

        try:
            current = int('0' * self.bit_count, 2)
            bit = int('0' * (self.bit_count - 1) + '1', 2)

            self.latch_pin.value = True
            for i in range(self.bit_count):
                self.clock_pin.value = False
                if self.data_pin.value:
                    current |= bit
                self.clock_pin.value = True
                bit <<= 1
            self.latch_pin.value = False
            self.pressed |= (self.last & current)
            self.last = current
        except RuntimeError as e:
            print("Error reading controller buttons", str(e))
        return self.pressed

    def wasPressed(self, b):
        if (self.pressed & b):
            return True
        else:
            return False


# This function allows a user to manage joystick handling themselves.
# From http://www.mimirgames.com/articles/games/joystick-input-and-using-deadbands/
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
    def readJoystickAxis(self):
        pinValue = self.pin.value
        send_dv_state(self.name, pinValue)
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
def send_dv_state(_pin_name, _pin_state):
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

def send_dv_i2c():
    send_dv_state("GP20", "P")
    send_dv_state("GP21", "P")

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
        c = 100 - int(min(math.pow(abs(a - b), 0.75), 100))
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

# find the index of a value in a list that is closest to the given value
def find_closest_in_list(target, lst, conf_thd, comp_func, find_value): 
    _best_index = -1 
    _best_score = 1e20 if comp_func == numberCompare else 0 
    for i, value in enumerate(lst):
        _score = find_in_list_compare(target, value, comp_func) 
        if (_score >= conf_thd and comp_func != numberCompare) or (_score < _best_score and comp_func == numberCompare): 
            _best_index = i 
            _best_score = _score
    if _best_index == -1 and find_value: 
        return None 
    elif find_value: 
        return lst[_best_index] 
    else: 
        return _best_index + 1

# used for comapring numerical values in a list
def find_in_list_compare(target, value, comp_func): 
    return comp_func(target, value) if comp_func != numberCompare else abs(target - value)

# helper function for graphing number values
def piperGraphNumbers(graph_values):
    print(chr(17), 'G', ','.join(graph_values), chr(17), end='')
    
# helper function for graphing a color value (tuple)
def piperGraphColor(color_value):
    print(chr(17), 'C', str(color_value), chr(17), end='')

# used to generate an RGB color value form a single integer 0-255
def piperColorWheel(hue_value, bright_value=100):
    bright_value = min(bright_value, 100) / 100.0
    _hue_value = colorwheel(int(_hue_value) & 255)
    _bv_r = int(((_hue_value) & 255) * bright_value) & 255
    _bv_g = int(((_hue_value >> 8) & 255) * bright_value) & 255
    _bv_b = int(((_hue_value >> 16) & 255) * bright_value) & 255
    return (_bv_r, _bv_g, _bv_b)

def randomColor(bright_value):
    if bright_value is None:
        return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
    else:
        return piperColorWheel(random.randint(0, 255), bright_value)
