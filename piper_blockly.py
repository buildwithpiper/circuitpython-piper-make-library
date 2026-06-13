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
try:
    import piper_heart_sensor
except:
    pass
try:
    import piper_motor_module
except:
    pass
try:
    from piper_lightshow import Lightshow, PixBuffer
except:
    pass
try:
    import piper_radio_module
except:
    pass
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
    """
    Enables or disables the Digital View serial output communication link.
    
    Args:
        state (bool): True to enable serial telemetry updates, False to disable.
    """
    global digital_view
    digital_view = state

################################################################################
# Hardware Interfacing Classes
################################################################################

class piperPin:
    """
    Manages basic General Purpose Input/Output (GPIO) pins configured as 
    Digital Input/Output, Analog Input, or PWM Voltage Output.
    """
    def __init__(self, pin, name=None, type='Digital'):
        """
        Initializes a generic GPIO pin.

        Args:
            pin (microcontroller.Pin): The physical hardware pin identifier.
            name (str, optional): User-defined name for tracking. Defaults to pin ID digits.
            type (str): Operational mode: 'Digital', 'Analog', or 'Voltage' (PWM).
        """
        if (not name):
            name = str(pin)[6:10]
        if type == 'Digital':
            self.pin = DigitalInOut(pin)
        elif type == 'Analog':
            self.pin = AnalogIn(pin)
        elif type == 'Voltage':
            self.pin = pwmio.PWMOut(pin, frequency=5000, duty_cycle=0)
        self.name = name
        self.type = type

    def setPin(self, pin_state):
        """
        Drives a pin configured as OUTPUT or adjusts a PWM duty cycle.

        Args:
            pin_state (bool/int/float): Logic level (True/False) for 'Digital' pins,
                                        or a float value (0.0 to 3.3) for 'Voltage' pins.
        """
        if (self.type == 'Voltage'):
            pin_value = max(min(float(pin_state), 3.3), 0) / 3.3
            self.pin.duty_cycle = int(65535 * pin_value)
            send_dv_state(self.name, pin_value)
        else:
            self.pin.direction = Direction.OUTPUT
            self.pin.value = pin_state
            send_dv_state(self.name, int(pin_state))

    def checkPin(self, pin_pull):
        """
        Configures a digital pin as an INPUT, applies internal pull resistance, and reads state.

        Args:
            pin_pull (digitalio.Pull): Pull configuration: Pull.UP, Pull.DOWN, or None.

        Returns:
            bool: The logic state of the input pin (True for High, False for Low).
        """
        self.pin.direction = Direction.INPUT
        self.pin.pull = pin_pull
        pin_value = self.pin.value
        send_dv_state(self.name, int(pin_value))
        return pin_value

    def readVoltage(self):
        """
        Reads an analog input pin and scales the raw value to a physical voltage level.

        Returns:
            float: Voltage range from 0.0 to 3.3 Volts.
        """
        pin_value = self.pin.value / 65536
        send_dv_state(self.name, pin_value)
        return pin_value * 3.3


class piperServoPin:
    """
    Controls a standard RC servo motor using hardware PWM signals.
    """
    def __init__(self, pin, name=None):
        """
        Initializes a hardware pin for 50Hz RC Servo motor control.

        Args:
            pin (microcontroller.Pin): The physical hardware pin identifier.
            name (str, optional): User-defined name for tracking.
        """
        if (not name):
            name = str(pin)[6:10]
        self.pwm = pwmio.PWMOut(pin, duty_cycle=0, frequency=50)
        self.pin = servo.Servo(self.pwm, min_pulse=580, max_pulse=2350)
        self.name = name

    def setServoAngle(self, a):
        """
        Sets the rotation angle of the servo motor.

        Args:
            a (int/float/None): Target angle in degrees (typically 0-180). 
                                Set to None to release/de-energize the servo hold.
        """
        send_dv_state(self.name, "P")
        try:
            if a == None:
                self.pin.fraction = None
            else:
                self.pin.angle = a
        except RuntimeError as e:
            print("Error setting servo angle", str(e))

    def setServoFraction(self, f):
        """
        Sets the position of the servo motor using a normalized fractional range.

        Args:
            f (float): Normalization fraction between 0.0 (minimum) and 1.0 (maximum).
        """
        send_dv_state(self.name, "P")
        try:
            self.pin.fraction = f
        except RuntimeError as e:
            print("Error setting servo position", str(e))


class piperCapSensePin:
    """
    Manages touch and proximity capabilities using hardware capacitive touch sensing pins.
    """
    def __init__(self, pin, name=None):
        """
        Initializes a capacitive touch pin.

        Args:
            pin (microcontroller.Pin): Touch-capable microcontroller hardware pin.
            name (str, optional): User-defined name for tracking.
        """
        if (not name):
            name = str(pin)[6:10]
        self.pin = TouchIn(pin)
        self.name = name

    def readCapSenseValue(self):
        """
        Reads the raw touch interface baseline value and normalizes it.

        Returns:
            int/None: Normalized touch sensing value, or None if a hardware fault occurs.
        """
        try:
            d = self.pin.raw_value
            send_dv_state(self.name, float(max(min(d / 10000, 10000), 0)))
        except RuntimeError as e:
            d = None
            print("Error reading capacitive sense value", str(e))
        return d


class piperDistanceSensorPin:
    """
    Controls an HC-SR04 or compatible ultrasonic distance / rangefinder sensor.
    """
    def __init__(self, pin, name=None):
        """
        Initializes an ultrasonic range finder on a single communication pin.

        Args:
            pin (microcontroller.Pin): Microcontroller pin linked to the sensor.
            name (str, optional): User-defined name for tracking.
        """
        if (not name):
            name = str(pin)[6:10]
        self.pin = piper_range_finder.PiperUltrasonicRanger(pin)
        self.name = name

    def readDistanceSensor(self):
        """
        Fires an ultrasonic pulse and calculates the distance to the nearest target.

        Returns:
            float/None: Distance value in centimeters, or None if reading fails/times out.
        """
        send_dv_state(self.name, "P")
        try:
            d = self.pin.distance
        except RuntimeError as e:
            d = None
            print("Error reading distance sensor", str(e))
        return d


class piperTemperatureSensor:
    """
    Interfaces with an MCP9808 high-accuracy digital I2C temperature sensor.
    """
    def __init__(self, i2c_bus):
        """
        Initializes the temperature sensor on the designated shared I2C bus channel.

        Args:
            i2c_bus (busio.I2C): The active, instantiated I2C bus object.
        """
        self.temperature_sensor = adafruit_mcp9808.MCP9808(i2c_bus)

    def readTemperatureSensor(self):
        """
        Queries the MCP9808 hardware sensor register for ambient environment readings.

        Returns:
            float: Ambient temperature measured in degrees Celsius.
        """
        send_dv_i2c()
        return self.temperature_sensor.temperature


class piperColorSensor:
    """
    Interfaces with a TCS34725 digital I2C RGB color and light sensor.
    """
    def __init__(self, i2c_bus):
        """
        Initializes the RGB sensor over an active I2C bus interface.

        Args:
            i2c_bus (busio.I2C): The active, instantiated I2C bus object.
        """
        self.color_sensor = adafruit_tcs34725.TCS34725(i2c_bus)
        self.color_sensor.gain = 60
        self.mult = pow((128/60), 0.6)

    def readColorSensor(self):
        """
        Reads raw RGB color values and maps them into human-normalized RGB bounds.

        Returns:
            tuple: An (R, G, B) tuple where each index contains an integer scaled 0 to 255.
        """
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
        """
        Adjusts the structural light integration amplification gain of the color sensor.

        Args:
            val (int): Amplification gain target. Acceptable values: 1, 4, 16, or 60.
        """
        self.mult = pow((128/val), 0.6)
        self.color_sensor.gain = val

    def read(self):
        """
        Alias wrapper for readColorSensor to standardize data access across peripherals.

        Returns:
            tuple: Normalized (R, G, B) integer color tuple from 0 to 255.
        """
        return self.readColorSensor()


class piperMotionSensor:
    """
    Interfaces with an MPU6050 6-axis I2C Inertial Measurement Unit (IMU).
    Tracks acceleration, rotational speed, tilt, roll, pitch, and yaw.
    """
    def __init__(self, i2c_bus, address=0x69):
        """
        Initializes the MPU6050 IMU motion tracker.

        Args:
            i2c_bus (busio.I2C): The active, instantiated I2C bus object.
            address (int, optional): Hardware device address on the I2C line. Defaults to 0x69.
        """
        self.motion_sensor = adafruit_mpu6050.MPU6050(i2c_bus, address=address)

    def readMotionSensor(self):
        """
        Queries the IMU hardware registers and derives spatial dynamics telemetry attributes.
        Updates internal object attributes: acc_x/y/z, gyro_x/y/z, temp, roll, pitch, yaw.
        """
        send_dv_i2c()
        self.acc_x, self.acc_y, self.acc_z = self.motion_sensor.acceleration
        self.gyro_x, self.gyro_y, self.gyro_z = self.motion_sensor.gyro
        self.temp = self.motion_sensor.temperature
        self.roll = math.atan2(self.acc_y, self.acc_z) * 180 / math.pi
        self.pitch = math.atan2(self.acc_z, self.acc_x) * 180 / math.pi
        self.yaw = math.atan2(self.acc_x, self.acc_y) * 180 / math.pi

    def read(self):
        """
        Alias wrapper for readMotionSensor to standardise data polling across submodules.
        """
        self.readMotionSensor()


class piperHeartSensor:
    """
    Interfaces with a digital I2C photoplethysmogram (PPG) optical biometric pulse oximeter.
    """
    def __init__(self, i2c_bus):
        """
        Initializes the heart rate monitor module on the active shared I2C bus.

        Args:
            i2c_bus (busio.I2C): The active, instantiated I2C bus object.
        """
        self.heart_sensor = piper_heart_sensor.PiperHeartSensor(i2c_bus, smoothing=4, channel=0)
        self.raw_value = 0
        self.heart_rate = -1

    def readHeartSensor(self):
        """
        Polls the heart rate sensor, updating biometric signal and pulse frequency attributes.
        Updates internal object attributes: raw_value, heart_rate (-1 if invalid or acquiring).
        """
        send_dv_i2c()
        self.raw_value = self.heart_sensor.read_sensor()
        if (self.heart_sensor.heart_rate == None):
            self.heart_rate = -1
        else:
            self.heart_rate = self.heart_sensor.heart_rate

    def read(self):
        """
        Alias wrapper for readHeartSensor to standardise tracking loop sequences.
        """
        self.readHeartSensor()


class piperNeoPixels:
    """
    Drives customizable digital strips, matrices, or rings of addressable WS2812B NeoPixel LEDs.
    """
    def __init__(self, pin, name=None, pixel_count=10):
        """
        Initializes a NeoPixel addressable driver sequence.

        Args:
            pin (microcontroller.Pin): Hardware output control pin.
            name (str, optional): User-defined name for tracking.
            pixel_count (int): Total number of sequential addressable LEDs in the circuit array.
        """
        if (not name):
            name = str(pin)[6:10]
        self.pixels = neopixel.NeoPixel(pin, pixel_count, brightness=0.6, auto_write=False)
        self.pin = pin
        self.name = name

    def fill(self, color):
        """
        Sets all pixels in the array buffer to a uniform color index without executing immediate render.

        Args:
            color (tuple/int): A 3-tuple representing (R, G, B) values (0-255), or hex code.
        """
        self.pixels.fill(color)

    def show(self):
        """
        Transmits data out to physical elements, forcing structural updates on the layout string.
        """
        send_dv_state(self.name, "P")
        self.pixels.show()


class piperLightshow:
    """
    Controls an 8x8 LED matrix display driver run continuously via a dedicated SPI interface pipeline.
    The Lightshow module must be connected to SPI0 (GP17, GP18, GP19).
    """
    def __init__(self):
        """
        Initializes the SPI matrix lightshow display controller and clear configuration buffers.
        """
        self.lightshow = Lightshow()
        self.pix = PixBuffer()

    def from_text(self, string, color=63, bgcolor=0, colors=None):
        """
        Renders a textual string sequence into pixel data layouts ready for projection.

        Args:
            string (str): Character array to format.
            color (int): Main element character color byte pattern. Defaults to 63.
            bgcolor (int): Environment background depth tint mask byte. Defaults to 0.
            colors (list, optional): Direct color array palette index parameters.
        """
        return self.pix.from_text(string, color, bgcolor, colors)

    def from_grid(self, lines):
        """
        Maps structural configurations using structural array formats.

        Args:
            lines (list): Structural list of string matrix values for mapping raw graphics.
        """
        return self.pix.from_grid(lines)

    def pixel(self, x, y, color=None):
        """
        Sets or queries the status parameters of an isolated target coordinate cell.

        Args:
            x (int): Horizontal column layout position index map (0 to 7).
            y (int): Vertical row layout position index map (0 to 7).
            color (int, optional): Set to a byte color code value, or leave empty to query existing state.

        Returns:
            int/None: Byte value profile of the coordinates if reading data, else None.
        """
        __pixel = self.pix.pixel(x, y, color)
        if __pixel is not None:
            return __pixel

    def box(self, color, x=0, y=0, width=None, height=None):
        """
        Draws a rectangular configuration array inside pixel boundaries.

        Args:
            color (int): Byte index value structure for filling bounds.
            x (int): Horizontal column start layout pointer. Defaults to 0.
            y (int): Vertical row start layout pointer. Defaults to 0.
            width (int, optional): Max column width range limits. Defaults to matrix limits.
            height (int, optional): Max row height range limits. Defaults to matrix limits.
        """
        self.pix.box(color, x, y, width, height)

    def clear(self):
        """
        Flushes and blanks the 8x8 structural element layout grid array completely.
        """
        self.pix.box(0, 0, 0, 8, 8)

    def draw(self, source, dx=0, dy=0, x=0, y=0, width=None, height=None, key=None):
        """
        Copies structural frame assets from a source buffer configuration onto the presentation stack.

        Args:
            source (PixBuffer): Image source frame asset reference.
            dx (int): Display canvas offset target alignment pointer for horizontal elements.
            dy (int): Display canvas offset target alignment pointer for vertical elements.
            x (int): Horizontal origin reference clip bound on source buffer.
            y (int): Vertical origin reference clip bound on source buffer.
            width (int, optional): Extent bounds configuration width index.
            height (int, optional): Extent bounds configuration height index.
            key (int, optional): Color key mask index used to treat a value as transparent.
        """
        self.pix.draw(source, dx, dy, x, y, width, height, key)

    def send_dv_lightshow(self, pixel_array):
        """
        Sends telemetry frame data structures via serial interfaces if system view is active.

        Args:
            pixel_array (list/bytearray): Raw structural canvas profile layout index values.
        """
        global digital_view
        if (digital_view == True):
            __base_str = 'ABDEFHIJKMNOQSTUVWXYZ{}[]=+:0123456789abcdefghijklmnopqrstuvwxyz'
            print(chr(17), 'L', ''.join([__base_str[__i] for __i in list(pixel_array)]), chr(17))
        
    def show(self):
        """
        Pushes structural memory layers down to hardware lines to update the active matrix LEDs.
        """
        self.lightshow.show(self.pix)
        self.send_dv_lightshow(self.pix.buffer)

    def format_color(self, color):
        """
        Translates normal standard standard RGB data types down to optimized hardware bit patterns.

        Args:
            color (tuple): Standard 3-tuple containing (R, G, B) data types.

        Returns:
            int: 8-bit encoded equivalent layout matching current driver parameters.
        """
        return self.lightshow.rgb_to_byte(color)


class piperMotorModule:
    """
    Manages dual DC motors and multi-channel servo subassemblies using an I2C coprocessor bridge.
    """
    def __init__(self, i2c_bus):
        """
        Initializes a hardware control profile for tracking motor systems.

        Args:
            i2c_bus (busio.I2C): The active, instantiated I2C bus object.
        """
        self.motor_module = piper_motor_module.PiperMotorModule(i2c_bus)

    def coast(self, motor=0):
        """
        Disconnects terminal power channels on a target motor to let it free-spin to a halt.

        Args:
            motor (int): Specific motor drive profile instance index (0 or 1). Defaults to 0.
        """
        send_dv_i2c()
        self.motor_module.coast(motor)

    def brake(self, motor=0):
        """
        Short-circuits terminal pairings on a target motor, creating counter-electromotive braking force.

        Args:
            motor (int): Specific motor drive profile instance index (0 or 1). Defaults to 0.
        """
        send_dv_i2c()
        self.motor_module.brake(motor)

    def stop(self):
        """
        Cuts power output streams across all active DC drive motors simultaneously (Coasts).
        """
        send_dv_i2c()
        self.motor_module.stop()

    def set_speed(self, motor=0, speed=0):
        """
        Sets the proportional drive speed and directional target of a DC motor channel.

        Args:
            motor (int): Specific motor drive profile instance index (0 or 1). Defaults to 0.
            speed (int/float): Speed reference indicator between -100 (Full Reverse) and 100 (Full Forward).
        """
        send_dv_i2c()
        self.motor_module.set_speed(motor, speed)

    def servo_angle(self, servo=0, angle=0):
        """
        Commands an auxiliary hobby servo channel to a specific angular position.

        Args:
            servo (int): Target extension servo pin index on the module board (0 to 3). Defaults to 0.
            angle (int/float): Goal position calculated in degrees (0 to 180). Defaults to 0.
        """
        send_dv_i2c()
        self.motor_module.servo_angle(servo, angle)

    def servo_stop(self, servo=0):
        """
        Disables active PWM tracking loops on a servo channel to release holding torque and conserve power.

        Args:
            servo (int): Target extension servo pin index on the module board (0 to 3). Defaults to 0.
        """
        send_dv_i2c()
        self.motor_module.servo_stop(servo)


# Constants associated with the Piper Radio Module
RADIO_COLOR_SENSOR = 0
RADIO_TEMP_SENSOR = 1
RADIO_MOTION_SENSOR = 2
RADIO_HEART_SENSOR = 3
RADIO_MOTOR_MODULE = 4

RADIO_GPIO_INPUT = 0
RADIO_GPIO_INPUT_PULLUP = 4
RADIO_GPIO_INPUT_PULLDOWN = 8
RADIO_GPIO_INPUT_ANALOG = 2
RADIO_GPIO_INPUT_TOUCH = 16
RADIO_GPIO_INPUT_RANGE = 32
RADIO_GPIO_OUTPUT_DIGITAL = 1
RADIO_GPIO_OUTPUT_ANALOG = 3
RADIO_GPIO_OUTPUT_SERVO = 33

RADIO_ALLOW_GPIO = 0xFE00
RADIO_ALLOW_MOTOR = 0x2
RADIO_ALLOW_LED = 0x1
RADIO_ALLOW_ALL = 0xFE03

class piperRadioModule:
    """
    Manages wireless networking protocols over an I2C sub-transceiver layout link.
    Enables remote component tracking and sensory bridge links over sub-GHz/2.4GHz bands.
    """
    def __init__(self, i2c_bus):
        """
        Initializes the communication adapter on local hardware interfaces.

        Args:
            i2c_bus (busio.I2C): The active, instantiated I2C bus object.
        """
        self.radio_module = piper_radio_module.piper_radio_module(i2c_bus)

    def read_sensor(self, peer, module_type=1, value_index=0):
        """
        Queries a remote peer device for automated telemetry parameters over wireless networks.

        Args:
            peer (int/str): Network ID reference code identifying the target device.
            module_type (int): Peripheral sensor constant archetype indicator flag. Defaults to 1.
            value_index (int): Parameter data type offset selection code. Defaults to 0.

        Returns:
            float/int/tuple: Telemetry response structural output provided by the remote hardware target.
        """
        send_dv_i2c()
        return self.radio_module.read_sensor(peer, module_type, value_index)
        
    def setup_gpio(self, peer, gpio_pin, pin_type):
        """
        Configures remote hardware pin profiles on a wireless network endpoint.

        Args:
            peer (int/str): Network ID reference code identifying the target node.
            gpio_pin (int): Pin layout number on the remote endpoint device.
            pin_type (int): Operational role selector constant (e.g., RADIO_GPIO_INPUT_PULLUP).
        """
        send_dv_i2c()
        self.radio_module.setup_gpio(peer, gpio_pin, pin_type)
        
    def read_gpio(self, peer, gpio_pin):
        """
        Reads input states from a physical pin located on a remote wireless endpoint node.

        Args:
            peer (int/str): Network ID reference code identifying the target node.
            gpio_pin (int): Target pin identifier layout number on the peer node.

        Returns:
            int/float: Logic states or analog evaluation results sent across network bridges.
        """
        send_dv_i2c()
        return self.radio_module.read_gpio(peer, gpio_pin)

    def write_gpio(self, peer, gpio_pin, value):
        """
        Drives the pin output states of a remote node over wireless network channels.

        Args:
            peer (int/str): Network ID reference code identifying the target node.
            gpio_pin (int): Physical pin index number located on the target node.
            value (int/bool): Output target state, driving logic values high or low.
        """
        send_dv_i2c()
        self.radio_module.write_gpio(peer, gpio_pin, value)

    def write_motor_module(self, peer, gpio_pin, value):
        """
        Sends drive commands to motor adapters wired directly onto remote wireless endpoint nodes.

        Args:
            peer (int/str): Network ID reference code identifying the target node.
            gpio_pin (int): Pin or channel sub-index identifying motor paths on the target node.
            value (int): Operational speed or directive payload parameters.
        """
        send_dv_i2c()
        self.radio_module.write_motor_module(peer, gpio_pin, value)

    def set_led_color(self, peer, color):
        """
        Changes color values on addressable pixel displays wired to remote wireless nodes.

        Args:
            peer (int/str): Network ID reference code identifying the target node.
            color (tuple): Standard 3-tuple formatting layout containing (R, G, B) parameters.
        """
        send_dv_i2c()
        self.radio_module.set_led_color(peer, color)
        
    def allow_peer(self, mask, allow):
        """
        Configures device authentication filters to permit or block structural wireless connections.

        Args:
            mask (int): Network bitmask filter defining device categories or sub-classes.
            allow (bool): True to whitelist and accept peer requests, False to discard them.
        """
        send_dv_i2c()
        self.radio_module.allow_peer(mask, allow)
        
    def get_address(self):
        """
        Queries the device's hardware MAC identifier layer or network address assignment.

        Returns:
            int/str: Network address assignment identification tag.
        """
        send_dv_i2c()
        return self.radio_module.get_address()
        
    def set_peer_address(self, peer, address):
        """
        Maps a structural tracking shortcut index directly onto a long-form wireless hardware address.

        Args:
            peer (int): Shorthand structural indexing label (e.g., node slot tracker).
            address (int/str): Long-form network target address assignment identity value.
        """
        send_dv_i2c()
        self.radio_module.set_peer_address(peer, address)

    def send_message(self, peer, message):
        """
        Transmits raw string or byte message streams to a target node on the network.

        Args:
            peer (int/str): Network ID reference code identifying the target node.
            message (str): Text payload data to transmit over wireless lines.
        """
        send_dv_i2c()
        self.radio_module.send_message(peer, message)

    def get_message(self):
        """
        Fetches the oldest unread text message payload available in the wireless receive queue.

        Returns:
            str/None: Received text string data packet, or None if the queue is empty.
        """
        send_dv_i2c()
        return self.radio_module.get_message()

    def get_firmware_version(self):
        """
        Queries the integrated network processor module to read its internal firmware version.

        Returns:
            str/float: Firmware version build stamp identification code.
        """
        send_dv_i2c()
        return self.radio_module.get_firmware_version()
    
    def get_send_success(self):
        """
        Checks if the last transmitted data packet reached its destination successfully.

        Returns:
            bool: True if an ACK message was returned by the receiver node, else False.
        """
        send_dv_i2c()
        return self.radio_module.get_send_success()

    def channel_analysis(self):
        """
        Scans the local radio frequency spectrum to assess signal interference and track link health.

        Returns:
            list/dict: Received Signal Strength Indication (RSSI) profiling metrics.
        """
        send_dv_i2c()
        return self.radio_module.channel_analysis()


# Constants associated with the Piper Make Controller Button Matrix Configurations
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

ANY_BUTTON = const(64767)   # Bitmask filter for standard 14-button layouts
ANY_BUTTON_16 = const(65535) # Bitmask filter for extended 16-button layouts

class piperControllerPins:
    """
    Interfaces with a shift-register based gamepad button controller matrix (up to 16 buttons).
    Uses parallel-in, serial-out hardware lines (Clock, Data, Latch) to parse button state updates.
    """
    def __init__(self, clock_pin, data_pin, latch_pin, clock_name=None, data_name=None, latch_name=None):
        """
        Initializes digital operational lines for shift register array sequencing.

        Args:
            clock_pin (microcontroller.Pin): Output timing synchronization line.
            data_pin (microcontroller.Pin): Input digital bit stream line.
            latch_pin (microcontroller.Pin): Output state preservation control line.
            clock_name (str, optional): User-defined name for structural debugging.
            data_name (str, optional): User-defined name for structural debugging.
            latch_name (str, optional): User-defined name for structural debugging.
        """
        self.clock_pin = DigitalInOut(clock_pin)
        self.data_pin = DigitalInOut(data_pin)
        self.latch_pin = DigitalInOut(latch_pin)

        self.clock_pin.direction = Direction.OUTPUT
        self.data_pin.direction = Direction.INPUT
        self.latch_pin.direction = Direction.OUTPUT

        if (not clock_name):
            clock_name = str(clock_pin)[6:10]
        if (not data_name):
            data_name = str(data_pin)[6:10]
        if (not latch_name):
            latch_name = str(latch_pin)[6:10]
        self.clock_name = clock_name
        self.data_name = data_name
        self.latch_name = latch_name

        self.last = 0
        self.pressed = 0
        self.released = 0

    def readButtons(self):
        """
        Latches and shifts in the current 16-bit button state from the gamepad.
        Tracks button down/up transitions since the last read cycle.

        Returns:
            int: A 16-bit packed bitmask representing the real-time state of all buttons.
        """
        send_dv_state(self.clock_name, "P")
        send_dv_state(self.data_name, "P")
        send_dv_state(self.latch_name, "P")

        try:
            current = 0
            bit = 1
            self.latch_pin.value = True
            for i in range(16):
                self.clock_pin.value = False
                if self.data_pin.value:
                    current |= bit
                self.clock_pin.value = True
                bit <<= 1
            self.latch_pin.value = False
            self.pressed |= ((~self.last) & current)
            self.released |= (self.last & (~current))
            self.last = current
        except RuntimeError as e:
            print("Error reading controller buttons", str(e))
        return self.last

    def isPressed(self, b):
        """
        Checks if a specific button is currently held down.

        Args:
            b (int): Button bitmask constant (e.g., BUTTON_1).

        Returns:
            bool: True if the button is active, False if open.
        """
        if (self.last & b):
            return True
        else:
            return False

    def wasPressed(self, b):
        """
        Checks if a button was pressed down since the last poll, clearing its latch state.

        Args:
            b (int): Button bitmask constant (e.g., BUTTON_1).

        Returns:
            bool: True if a button press event occurred, False otherwise.
        """
        if (self.pressed & b):
            self.pressed &= (~b)
            return True
        else:
            return False

    def wasReleased(self, b):
        """
        Checks if a button transitioned from pressed to open since the last poll, clearing its latch state.

        Args:
            b (int): Button bitmask constant (e.g., BUTTON_1).

        Returns:
            bool: True if a button release event occurred, False otherwise.
        """
        if (self.released & b):
            self.released &= (~b)
            return True
        else:
            return False


class piperJoystickAxis:
    """
    Manages an analog joystick coordinate axis with custom deadbands and cubic response curve scaling.
    Provides precise, granular motion tracking around the center axis (ideal for mouse emulation/HID controls).
    """
    def __init__(self, pin, name=None, outputScale=20.0, deadbandCutoff=0.1, weight=0.2):
        """
        Initializes an analog joystick axis parser.

        Args:
            pin (microcontroller.Pin): Analog input channel wired to the joystick potentiometer.
            name (str, optional): User-defined name for tracking.
            outputScale (float): Maximum mapped boundary output limit multiplier. Defaults to 20.0.
            deadbandCutoff (float): Dead-zone ratio around center (0.0 to 1.0) to filter drift. Defaults to 0.1.
            weight (float): Cubic calculation interpolation blend weight (0.0 to 1.0). Defaults to 0.2.
        """
        if (not name):
            name = str(pin)[6:10]
        self.name = name
        self.pin = AnalogIn(pin)
        self.outputScale = outputScale
        self.deadbandCutoff = deadbandCutoff
        self.weight = weight
        self.alpha = self._Cubic(self.deadbandCutoff)

    def _Cubic(self, x):
        """
        Applies a cubic transformation profile to smooth out low-end value response curves.

        Args:
            x (float): Normalized input scaling reference point.

        Returns:
            float: Exponentially balanced scaling adjustment value.
        """
        return self.weight * x ** 3 + (1.0 - self.weight) * x

    def _cubicScaledDeadband(self, x):
        """
        Filters out mechanical center-drift while smoothing the transition out of the dead zone.

        Args:
            x (float): Raw position ratio from -1.0 to 1.0.

        Returns:
            float: Processed translation factor from -1.0 to 1.0 (or 0 if within deadband).
        """
        if abs(x) < self.deadbandCutoff:
            return 0
        else:
            return (self._Cubic(x) - (copysign(1,x)) * self.alpha) / (1.0 - self.alpha)

    def readJoystickAxis(self):
        """
        Reads raw analog inputs, applies deadband thresholds, scales via response curve,
        and returns an integer target value.

        Returns:
            int: Filtered control output value within the range [-outputScale, +outputScale].
        """
        pin_value = self.pin.value
        send_dv_state(self.name, pin_value / 65535)
        return int(self._cubicScaledDeadband((pin_value / 2 ** 15) - 1) * self.outputScale)


################################################################################
# Blockly Support & Telemetry Helper Functions
################################################################################

def isNumber(n):
    """
    Validates if a target data type can be interpreted as a number, casting string variants if needed.

    Args:
        n (any): Target variable payload to test.

    Returns:
        int/float/int(0): Parsed float/int value if numeric, or integer 0 if evaluation fails.
    """
    if not (type(n) is int or type(n) is float):
        try:
            n = float(n)
        except:
            return 0
    return n

def consoleClear():
    """
    Transmits terminal clear control bytes to clear the connected interface console screen.
    """
    print(chr(16), end="")

def consolePosition(x, y):
    """
    Transmits cursor placement formatting sequences to position text output at specific coordinates.

    Args:
        x (int): Horizontal column coordinate (0 to 255).
        y (int): Vertical row coordinate (0 to 255).
    """
    x = (min(max(int(x), 0), 255))
    y = (min(max(int(y), 0), 255))
    print(chr(17), 'P', str(x) + ',' + str(y), chr(17), end='')

def send_dv_state(_pin_name, _pin_state):
    """
    Internal telemetry helper that transmits terminal pin update streams when a pin value changes.

    Args:
        _pin_name (str): Identifier key matching hardware pin configurations.
        _pin_state (any): Real-time parameter value or active status byte profile.
    """
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
    """
    Sends a terminal instruction that indicates I2C data has been sent.
    """
    send_dv_state("GP20", "P")
    send_dv_state("GP21", "P")

def playSound(soundName): 
    """
    Sends explicit terminal instruction markers to trigger audio playback on a connected computer.

    Args:
        soundName (str): Identifier matching target audio assets or synth scripts.
    """
    print(chr(19), soundName, chr(19), end="")

def shout(color, text):
    """
    Sends explicit instruction markers to display styled pop-up alerts on a connected host application.

    Args:
        color (str/int): Color configuration tag used to define notification borders.
        text (str): Notification string payload to print inside the alert window.
    """
    print(chr(18), str(color) + "|" + str(text), chr(18), end="")

def emojiCharacter(c):
    """
    Maps semantic emotion keywords directly onto designated proprietary interface terminal control codes.

    Args:
        c (str): Emotion keyword ('happy', 'sad', 'thinking', 'confused', 'in-love', 'quiet', 'suspicious', 'unhappy', 'bored', or 'surprised').

    Returns:
        str: Single-character control byte used to trigger asset rendering.
    """
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

def colorCompare(a, b):
    """
    Calculates a similarity percentage profile matching two distinct RGB color sequences.

    Args:
        a (tuple): First RGB baseline tuple (R, G, B) evaluated 0 to 255.
        b (tuple): Second RGB validation tuple (R, G, B) evaluated 0 to 255.

    Returns:
        int: Score ranging from 0 (completely opposite) to 100 (identical match).
    """
    try:
        c = 100 - int((abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])) * 20 / 153)
    except:
        return 0
    return c

def numberCompare(a, b):
    """
    Compares two numerical values using a power-scaled curve to determine relative similarity.

    Args:
        a (int/float): Value A.
        b (int/float): Value B.

    Returns:
        int: Proportional match confidence score from 0 (disparate) to 100 (identical).
    """
    try:
        c = 100 - int(min(math.pow(abs(a - b), 0.75), 100))
    except:
        return 0
    return c

def stringCompare(a, b):
    """
    Evaluates text structure intersection logic to generate a similarity score between two strings.

    Args:
        a (str): Text string instance A.
        b (str): Text string instance B.

    Returns:
        int: Intersection similarity index value evaluated from 0 to 100.
    """
    try:
        c = set(list(a))
        d = set(list(b))
        e = c.intersection(d)
        f = int((float(len(e)) / (len(c) + len(d) - len(e))) * 100)
    except:
        return 0
    return f

def mapValue(value, a, b, c, d):
    """
    Linear re-maps (scales) a value from its original operational range to a new target range.

    Args:
        value (float/int): Source value to scale.
        a (float/int): Lower limit of original range bounds.
        b (float/int): Upper limit of original range bounds.
        c (float/int): Lower limit of new destination range bounds.
        d (float/int): Upper limit of new destination range bounds.

    Returns:
        float: Normalized scaled value adjusted inside target boundaries.
    """
    return c + ((float(value - a) / float(b - a)) * (d - c))

def find_closest_in_list(target, lst, conf_thd, comp_func, find_value): 
    """
    Parses an array to extract elements or index pointers that most closely match a target criterion.

    Args:
        target (any): Search value target criteria baseline.
        lst (list): Search domain container index listing arrays.
        conf_thd (float/int): Match qualification floor limits required for acceptance.
        comp_func (function): Comparison handler pointer (e.g., colorCompare, numberCompare).
        find_value (bool): Set True to return the direct matching asset item; 
                           Set False to return its 1-based index location instead.

    Returns:
        any/int/None: Array target contents, 1-based layout index location, or None if no match passes.
    """
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

def find_in_list_compare(target, value, comp_func): 
    """
    Wrapper for parsing individual matching tasks during list scanning.

    Args:
        target (any): Base objective target parameter layout.
        value (any): Single array item element configuration being evaluated.
        comp_func (function): The dynamic comparison function being used.

    Returns:
        float/int: Similarity metric or absolute delta value.
    """
    return comp_func(target, value) if comp_func != numberCompare else abs(target - value)

def webcam_read_poses():
    """
    Sends a serial command request asking the host computer to parse real-time camera image skeleton pose data.
    """
    print(chr(17) + 'M |R' + chr(17), end='')

def webcam_get_pose(_val, _att):
    """
    Fetches processed webcam body telemetry parameters sent back from the host computer.

    Args:
        _val (str): Index label targeting a specific individual or coordinate item tracker.
        _att (str): Attribute key or joint node label mapping target details (e.g., 'x', 'y', 'nose').

    Returns:
        int: Physical alignment coordinates parsed along host application tracking arrays.
    """
    return int(input(chr(17) + 'M |W,' + _val + ',' + _att + chr(17)))

def webcam_get_color():
    """
    Fetches real-time color sampling data from the host computer's webcam feed.

    Returns:
        tuple: Evaluated (R, G, B) integer color sample mapping values from 0 to 255.
    """
    hex_str = input(chr(17) + 'M |E' + chr(17))
    return (int(hex_str[0:2], 16), int(hex_str[2:4], 16), int(hex_str[4:6], 16))

def mic_read_commands():
    """
    Sends a serial command request instructing the host computer to start polling microphone speech recognition.
    """
    print(chr(17) + 'M |C' + chr(17), end='')

def mic_get_command(_conf):
    """
    Fetches the parsed textual string representing recognized speech commands from the host computer.

    Args:
        _conf (float/int): Match baseline criteria confidence score tracking filters.

    Returns:
        str: Parsed word string text recognized by the host speech processing systems.
    """
    return input(chr(17) + 'M |S,' + str(_conf) + chr(17))

def fetch_url_data(_url):
    """
    Requests data from an external HTTP endpoint through the host computer's internet connection.

    Args:
        _url (str): Target web URL endpoint string to fetch.

    Returns:
        list: Decoded data split by commas.
    """
    return input(chr(17) + 'M |U,' + str(_url) + chr(17)).split(',')

def piperGraphNumbers(graph_values):
    """
    Sends a numerical dataset back to the host computer to be charted real-time on a graphical visualizer.

    Args:
        graph_values (list of str): Structured numbers formatted as strings to feed into tracking charts.
    """
    print(chr(17), 'G', ','.join(graph_values), chr(17), end='')
    
def piperGraphColor(color_value):
    """
    Sends an RGB color tuple back to the host computer to be displayed on a live color tracking visualizer.

    Args:
        color_value (tuple): Standard 3-tuple (R, G, B) target asset parameters evaluated 0 to 255.
    """
    print(chr(17), 'C', str(color_value), chr(17), end='')

def piperColorWheel(hue_value, bright_value=100):
    """
    Generates full structural standard RGB color tuples using standard raw 0-255 hue positioning numbers.

    Args:
        hue_value (int): Position wheel shift indicator step number (0 to 255).
        bright_value (int/float): Amplitude scaling ceiling limit percentage (0 to 100). Defaults to 100.

    Returns:
        tuple: Formatted (R, G, B) integer color match parameters from 0 to 255.
    """
    bright_value = min(bright_value, 100) / 100.0
    hue_value = colorwheel(int(hue_value) & 255)
    _bv_r = int(((hue_value) & 255) * bright_value) & 255
    _bv_g = int(((hue_value >> 8) & 255) * bright_value) & 255
    _bv_b = int(((hue_value >> 16) & 255) * bright_value) & 255
    return (_bv_r, _bv_g, _bv_b)

def randomColor(bright_value=None):
    """
    Generates a randomized color tuple profile, with optional brightness scaling limits.

    Args:
        bright_value (int/float, optional): Brightness percentage floor scale (0 to 100). 
                                             Leave empty for raw unweighted variations.

    Returns:
        tuple: Randomized (R, G, B) value settings evaluated from 0 to 255.
    """
    if bright_value is None:
        return (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
    else:
        return piperColorWheel(random.randint(0, 255), bright_value)
