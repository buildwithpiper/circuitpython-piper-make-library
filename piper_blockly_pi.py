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

import gpiozero
import math
from threading import Event, Lock
import adafruit_mcp9808
import adafruit_tcs34725
import adafruit_mpu6050
import piper_heart_sensor
#from touchio import TouchIn
import neopixel
from rainbowio import colorwheel
import time

try:
    from gpiozero.pins.pigpio import PiGPIOFactory
except ImportError:
    PiGPIOFactory = None

digital_view = True

piper_pin_states = []
piper_pin_names = []

Pull = {
    'UP': 'up',
    'DOWN': 'down',
}

def set_digital_view(state):
    global digital_view
    digital_view = state

################################################################################
# This class is for digital GPIO pins
#
class piperPin:
    def __init__(self, pin, name):
        self.pin = gpiozero.DigitalInputDevice(pin)
        self.name = name

    # Report the pin's state for use by the digital view
    #
    def reportPin(self, pinStr):
        global digital_view
        if (digital_view == True):
            if not pinStr:
                pinStr = float(self.pin.value)
            send_dv_state(self.name, pinStr)

    # Sets the pin to be an output at the specified logic level
    #
    def setPin(self, pinState):
        self.pin.output_with_state(pinState)
        self.reportPin(pinState)

    # Reads the pin by setting it to an input and setting it's pull-up/down and then returning its value
    # (Note that this means you can't use it to detect the state of output pins)
    #
    def checkPin(self, pinPull='floating'):
        self.pin.input_with_pull(pinPull)
        pinValue = self.pin.value
        self.reportPin(float(pinValue))
        return pinValue

    # Reads an analog voltage from the specified pin
    #
    """
    def readVoltage(self):
        pinValue = self.pin.value / 65536
        self.reportPin(pinValue)
        return pinValue * 3.3
    """

# This is specific to pins which are attached to a servo
# and we won't allow GPIO operations for now
#
class piperServoPin:
    def __init__(self, pin, name):
        # create a servo object on the control pin.
        self.pin = gpiozero.Servo(pin, min_pulse_width=0.580, max_pulse_width=2.350)
        self.name = name

    def setServoAngle(self, a):
        send_dv_state(self.name, "P")
        self.pin.value = a / 90 - 1

    def setServoFraction(self, f):
        send_dv_state(self.name, "P")
        self.pin.value = f * 2 - 1

# This is specific to pins which are used for capacitive sensing sensor
# and we won't allow GPIO operations for now
#
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
# and we won't allow GPIO operations for now
#
class piperDistanceSensorPin:
    """
    Extends :class:`SmoothedInputDevice` and represents an ultrasonic distance
    sensor.

    .. note::

        For improved accuracy, use the pigpio pin driver rather than the default
        RPi.GPIO driver (pigpio uses DMA sampling for much more precise edge
        timing). This is particularly relevant if you're using Pi 1 or Pi Zero.
        See :ref:`changing-pin-factory` for further information.
    """

    ECHO_LOCK = Lock()

    def __init__(
            self, echo, name, queue_len=9, max_distance=3,
            threshold_distance=0.35, partial=False, pin_factory=None):
        super(piperDistanceSensorPin, self).__init__(
            echo, pull_up=False, queue_len=queue_len, sample_wait=0.06,
            partial=partial, ignore=frozenset({None}), pin_factory=pin_factory
        )
        try:
            self.name = name
            if max_distance <= 0:
                raise ValueError('invalid maximum distance (must be positive)')
            self._max_distance = max_distance
            self.threshold = threshold_distance / max_distance
            self.speed_of_sound = 343.26 # m/s
            self._echo = Event()
            self._echo_rise = None
            self._echo_fall = None
            self.pin.edges = 'both'
            self.pin.bounce = None
            self.pin.when_changed = self._echo_changed
            self._queue.start()
        except:
            self.close()
            raise

        #if PiGPIOFactory is None or not isinstance(self.pin_factory, PiGPIOFactory):
            #print(
            #    'For more accurate readings, use the pigpio pin factory.'
            #    'See https://gpiozero.readthedocs.io/en/stable/api_input.html#distancesensor-hc-sr04 for more info'
            #)

    def close(self):
        super(piperDistanceSensorPin, self).close()

    @property
    def max_distance(self):
        """
        The maximum distance that the sensor will measure in meters. This value
        is specified in the constructor and is used to provide the scaling for
        the :attr:`~SmoothedInputDevice.value` attribute. When :attr:`distance`
        is equal to :attr:`max_distance`, :attr:`~SmoothedInputDevice.value`
        will be 1.
        """
        return self._max_distance

    @max_distance.setter
    def max_distance(self, value):
        if value <= 0:
            raise ValueError('invalid maximum distance (must be positive)')
        t = self.threshold_distance
        self._max_distance = value
        self.threshold_distance = t

    @property
    def threshold_distance(self):
        """
        The distance, measured in meters, that will trigger the
        :attr:`when_in_range` and :attr:`when_out_of_range` events when
        crossed. This is simply a meter-scaled variant of the usual
        :attr:`~SmoothedInputDevice.threshold` attribute.
        """
        return self.threshold * self.max_distance

    @threshold_distance.setter
    def threshold_distance(self, value):
        self.threshold = value / self.max_distance

    @property
    def distance(self):
        """
        Returns the current distance measured by the sensor in meters. Note
        that this property will have a value between 0 and
        :attr:`max_distance`.
        """
        return self.value * self._max_distance

    @property
    def value(self):
        """
        Returns a value between 0, indicating the reflector is either touching
        the sensor or is sufficiently near that the sensor can't tell the
        difference, and 1, indicating the reflector is at or beyond the
        specified *max_distance*.
        """
        send_dv_state(self.name, "P")
        return super(piperDistanceSensorPin, self).value

    @property
    def echo(self):
        """
        Returns the :class:`Pin` that the sensor's echo is connected to. This
        is simply an alias for the usual :attr:`~GPIODevice.pin` attribute.
        """
        return self.pin

    def _echo_changed(self, ticks, level):
        if level:
            self._echo_rise = ticks
        else:
            self._echo_fall = ticks
            self._echo.set()

    def _read(self):
        # Wait up to 50ms for the echo pin to fall to low (the maximum echo
        # pulse is 35ms so this gives some leeway); if it doesn't something is
        # horribly wrong (most likely at the hardware level)
        if self.pin.state:
            if not self._echo.wait(0.05):
                print('echo pin set high')
                return None
        self._echo.clear()
        self._echo_fall = None
        self._echo_rise = None
        # Obtain the class-level ECHO_LOCK to ensure multiple distance sensors
        # don't listen for each other's "pings"
        with piperDistanceSensorPin.ECHO_LOCK:
            # Fire the trigger
            self.pin.function = 'output'
            self.pin.state = True
            time.sleep(0.00001)
            self.pin.state = False
            time.sleep(0.00001)
            self.pin.function = 'input'

            # Wait up to 100ms for the echo pin to rise and fall (35ms is the
            # maximum pulse time, but the pre-rise time is unspecified in the
            # "datasheet"; 100ms seems sufficiently long to conclude something
            # has failed)
            if self._echo.wait(0.1):
                if self._echo_fall is not None and self._echo_rise is not None:
                    distance = (
                        self.pin_factory.ticks_diff(
                            self._echo_fall, self._echo_rise) *
                        self.speed_of_sound / 2.0)
                    return min(1.0, distance / self._max_distance)
                else:
                    # If we only saw the falling edge it means we missed
                    # the echo because it was too fast
                    return None
            else:
                # The echo pin never rose or fell; something's gone horribly
                # wrong
                print('no echo received')
                return None

    @property
    def in_range(self):
        return not self.is_active


class piperRCtimePin(gpiozero.SmoothedInputDevice):
    """
    Extends :class:`SmoothedInputDevice` and represents a resistive sensor or analog voltage.

    Connect a small resistor (220 or 330 ohm) to a GPIO Pin.  
    The other leg of the resistor is now the RCtime input.

    Resistive Sensor (potentiometer):
    Connect one leg of a resistive sensor to the 3V3 pin; connect one leg of a 1µF
    capacitor to a ground pin; connect the other leg of the resistive sensor and the other
    leg of the capacitor to the RCtime input. 
    
    Analog Voltage:
    connect one leg of a 1µF capacitor to a ground pin; connect the analog voltage source
    and the other leg of the capacitor to the RCtime input.
    Note, the max voltage must be 3.3V or less, and this circuit can only read down to 1.8V
    (Range of only 3.3V - 1.8V = 1.5V unfortunately.)

    This class repeatedly discharges
    the capacitor, then times the duration it takes to charge (which will vary
    according to the resistance or voltage.
    """
    def __init__(
            self, pin, name, queue_len=5, charge_time_limit=0.01,
            threshold=0.1, partial=False, pin_factory=None):
        super(piperRCtimePin, self).__init__(
            pin, pull_up=False, threshold=threshold, queue_len=queue_len,
            sample_wait=0.0, partial=partial, pin_factory=pin_factory)
        try:
            self.name = name
            self._charge_time_limit = charge_time_limit
            self._charge_time = None
            self._charged = Event()
            self.pin.edges = 'rising'
            self.pin.bounce = None
            self.pin.when_changed = self._cap_charged
            self._queue.start()
        except:
            self.close()
            raise

    @property
    def charge_time_limit(self):
        return self._charge_time_limit

    def _cap_charged(self, ticks, state):
        self._charge_time = ticks
        self._charged.set()

    def _read(self):
        # Drain charge from the capacitor
        self.pin.function = 'output'
        self.pin.state = False
        time.sleep(0.1)
        self._charge_time = None
        self._charged.clear()
        # Time the charging of the capacitor
        start = self.pin_factory.ticks()
        self.pin.function = 'input'
        self._charged.wait(self.charge_time_limit)
        if self._charge_time is None:
            return 0.0
        else:
            return 1.0 - min(1.0,
                (self.pin_factory.ticks_diff(self._charge_time, start) /
                self.charge_time_limit))

    @property
    def value(self):
        """
        Returns a value between 0 (dark) and 1 (light).
        """
        send_dv_state(self.name, "P")
        return super(piperRCtimePin, self).value


# The temperature sensor is attached to the I2C bus which can be shared
#
class piperTemperatureSensor:
    def __init__(self, i2c_bus):
        self.temperature_sensor = adafruit_mcp9808.MCP9808(i2c_bus)

    def readTemperatureSensor(self):
        send_dv_state("GP20", "P")
        send_dv_state("GP21", "P")
        return self.temperature_sensor.temperature

# The color sensor is attached to the I2C bus which can be shared
#
class piperColorSensor:
    def __init__(self, i2c_bus):
        self.color_sensor = adafruit_tcs34725.TCS34725(i2c_bus)
        self.color_sensor.gain = 60
        self.mult = pow((128/60), 0.6)

    def readColorSensor(self):
        send_dv_state("GP20", "P")
        send_dv_state("GP21", "P")

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
        send_dv_state("GP20", "P")
        send_dv_state("GP21", "P")

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
        send_dv_state("GP20", "P")
        send_dv_state("GP21", "P")

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
        send_dv_state(self.name, "P")
        self.pixels.show()
        

# constants associated with the Piper Make Controller
BUTTON_1 = (128)
BUTTON_2 = (64)
BUTTON_3 = (32)
BUTTON_4 = (16)

BUTTON_5 = (8)
BUTTON_6 = (4)
BUTTON_7 = (2)
BUTTON_8 = (1)
BUTTON_9 = (32768)
BUTTON_10 = (16384)

BUTTON_11 = (8192)
BUTTON_12 = (4096)
BUTTON_13 = (2048)
BUTTON_14 = (1024)

ANY_BUTTON = (64767)  # All of the bits used by the controller

# This is specific to pins which are attached to the Piper Make Controller
# and we won't allow GPIO operations for now
#
class piperControllerPins:
    def __init__(self, clock_pin, clock_name, data_pin, data_name, latch_pin, latch_name):
        self.clock_pin = gpiozero.OutputDevice(clock_pin, initial_value=True)
        self.data_pin = gpiozero.InputDevice(data_pin, pull_up=None)
        self.latch_pin = gpiozero.OutputDevice(latch_pin, initial_value=False)
        
        self.clock_name = clock_name
        self.data_name = data_name
        self.latch_name = latch_name
        self.bit_count = 16

        self.last = int('0' * self.bit_count, 2)
        self.pressed = self.last
        
        #self.gamepad = GamePadShift(self.clock_pin, self.data_pin, self.latch_pin, 16)

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
