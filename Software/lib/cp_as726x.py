# Combined AS726x Sensor Library
# Copyright (c) 2018 Timothy Garcia
# This version adds AS7261 and AS7263 support
#
# The MIT License (MIT)
#
# Copyright (c) 2017 Dean Miller for Adafruit Industries
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
"""
`adafruit_as726x`
====================================================

Driver for the AS726x spectral sensors

* Author(s): Dean Miller, Timothy Garcia
"""

import time
from adafruit_bus_device.i2c_device import I2CDevice
from micropython import const

try:
    import struct
except ImportError:
    import ustruct as struct

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_AS726x.git"

_AS726X_ADDRESS = const(0x49)

_AS726X_HW_VERSION = const(0x00)
_AS726X_FW_VERSION = const(0x02)
_AS726X_CONTROL_SETUP = const(0x04)
_AS726X_INT_T = const(0x05)
_AS726X_DEVICE_TEMP = const(0x06)
_AS726X_LED_CONTROL = const(0x07)

#for reading sensor data
_AS7262_V_HIGH = const(0x08)
_AS7262_V_LOW = const(0x09)
_AS7262_B_HIGH = const(0x0A)
_AS7262_B_LOW = const(0x0B)
_AS7262_G_HIGH = const(0x0C)
_AS7262_G_LOW = const(0x0D)
_AS7262_Y_HIGH = const(0x0E)
_AS7262_Y_LOW = const(0x0F)
_AS7262_O_HIGH = const(0x10)
_AS7262_O_LOW = const(0x11)
_AS7262_R_HIGH = const(0x12)
_AS7262_R_LOW = const(0x13)

_AS7262_V_CAL = const(0x14)
_AS7262_B_CAL = const(0x18)
_AS7262_G_CAL = const(0x1C)
_AS7262_Y_CAL = const(0x20)
_AS7262_O_CAL = const(0x24)
_AS7262_R_CAL = const(0x28)

#hardware registers
_AS726X_SLAVE_STATUS_REG = const(0x00)
_AS726X_SLAVE_WRITE_REG = const(0x01)
_AS726X_SLAVE_READ_REG = const(0x02)
_AS726X_SLAVE_TX_VALID = const(0x02)
_AS726X_SLAVE_RX_VALID = const(0x01)

#AS7261 Hardware Registers
_AS7261_X_HIGH =  const(0x08)
_AS7261_X_LOW = const(0x09)
_AS7261_Y_HIGH = const(0x0A)
_AS7261_Y_LOW = const(0x0B)
_AS7261_Z_HIGH = const(0x0C)
_AS7261_Z_LOW = const(0x0D)
_AS7261_NIR_HIGH = const(0x0E)
_AS7261_NIR_LOW = const(0x0F)
_AS7261_DARK_HIGH = const(0x10)
_AS7261_DARK_LOW = const(0x11)
_AS7261_CLEAR_HIGH = const(0x12)
_AS7261_CLEAR_LOW = const(0x13)
_AS7261_CAL_X = const(0x14)
_AS7261_CAL_Y = const(0x18)
_AS7261_CAL_Z = const(0x1C)
_AS7261_CAL_X_1931 = const(0x20)
_AS7261_CAL_Y_1931 = const(0x24)
_AS7261_CAL_UPRI = const(0x28)
_AS7261_CAL_VPRI = const(0x2C)
_AS7261_CAL_U = const(0x30)
_AS7261_CAL_V = const(0x34)
_AS7261_CAL_DUV = const(0x38)
_AS7261_CAL_LUX = const(0x3C)
_AS7261_CAL_CCT = const(0x40)


#AS7262 Hardware Registers
_AS7262_VIOLET = const(0x08)
_AS7262_BLUE = const(0x0A)
_AS7262_GREEN = const(0x0C)
_AS7262_YELLOW = const(0x0E)
_AS7262_ORANGE = const(0x10)
_AS7262_RED = const(0x12)
_AS7262_VIOLET_CALIBRATED = const(0x14)
_AS7262_BLUE_CALIBRATED = const(0x18)
_AS7262_GREEN_CALIBRATED = const(0x1C)
_AS7262_YELLOW_CALIBRATED = const(0x20)
_AS7262_ORANGE_CALIBRATED = const(0x24)
_AS7262_RED_CALIBRATED = const(0x28)

#AS7263 Hardware Registers
_AS7263_R = const(0x08)
_AS7263_S = const(0x0A)
_AS7263_T = const(0x0C)
_AS7263_U = const(0x0E)
_AS7263_V = const(0x10)
_AS7263_W = const(0x12)
_AS7263_R_CAL = const(0x14)
_AS7263_S_CAL = const(0x18)
_AS7263_T_CAL = const(0x1C)
_AS7263_U_CAL = const(0x20)
_AS7263_V_CAL = const(0x24)
_AS7263_W_CAL = const(0x28)

_AS726X_NUM_CHANNELS = const(6)

#pylint: disable=too-many-instance-attributes
#pylint: disable=too-many-public-methods
class Adafruit_AS726x(object):
    """AS726x spectral sensor.
       :param ~busio.I2C i2c_bus: The I2C bus connected to the sensor
       """

    MODE_0 = 0b00
    """Continuously gather samples of violet, blue, green and yellow. Orange and red are skipped
       and read zero."""

    MODE_1 = 0b01
    """Continuously gather samples of green, yellow, orange and red. Violet and blue are skipped
       and read zero."""

    MODE_2 = 0b10 #default
    """Continuously gather samples of all colors"""

    ONE_SHOT = 0b11
    """Gather a single sample of all colors and then stop"""

    GAIN = (1, 3.7, 16, 64)

    INDICATOR_CURRENT_LIMITS = (1, 2, 4, 8)

    DRIVER_CURRENT_LIMITS = (12.5, 25, 50, 100)

    def __init__(self, i2c_bus):
        self._driver_led = False
        self._indicator_led = False
        self._driver_led_current = Adafruit_AS726x.DRIVER_CURRENT_LIMITS.index(12.5)
        self._indicator_led_current = Adafruit_AS726x.INDICATOR_CURRENT_LIMITS.index(1)
        self._conversion_mode = Adafruit_AS726x.MODE_2
        self._integration_time = 0
        self._gain = Adafruit_AS726x.GAIN.index(1)
        self.buf2 = bytearray(2)

        self.i2c_device = I2CDevice(i2c_bus, _AS726X_ADDRESS)

        #reset device
        self._virtual_write(_AS726X_CONTROL_SETUP, 0x80)

        #wait for it to boot up
        time.sleep(1)

        #try to read the version reg to make sure we can connect
        version = self._virtual_read(_AS726X_HW_VERSION)

        #TODO: add support for other devices
        if version != 0x40 and version != 0x3F and version != 0x01:
            raise ValueError("device could not be reached or this device is not supported!")

        self._hw_read = version
        self.integration_time = 140
        self.conversion_mode = Adafruit_AS726x.MODE_2
        self.gain = 64

    @property
    def hw_version(self):
        """return hardware fw read"""
        return self._hw_read

    @property
    def driver_led(self):
        """True when the driver LED is on. False otherwise."""
        return self._driver_led

    @driver_led.setter
    def driver_led(self, val):
        val = bool(val)
        if self._driver_led == val:
            return
        self._driver_led = val
        enable = self._virtual_read(_AS726X_LED_CONTROL)
        enable &= ~(0x1 << 3)
        self._virtual_write(_AS726X_LED_CONTROL, enable | (val << 3))

    @property
    def indicator_led(self):
        """True when the indicator LED is on. False otherwise."""
        return self._indicator_led

    @indicator_led.setter
    def indicator_led(self, val):
        val = bool(val)
        if self._indicator_led == val:
            return
        self._indicator_led = val
        enable = self._virtual_read(_AS726X_LED_CONTROL)
        enable &= ~(0x1)
        self._virtual_write(_AS726X_LED_CONTROL, enable | val)

    @property
    def driver_led_current(self):
        """The current limit for the driver LED in milliamps. One of:

           - 12.5 mA
           - 25 mA
           - 50 mA
           - 100 mA"""
        return self._driver_led_current

    @driver_led_current.setter
    def driver_led_current(self, val):
        if val not in Adafruit_AS726x.DRIVER_CURRENT_LIMITS:
            raise ValueError("Must be 12.5, 25, 50 or 100")
        if self._driver_led_current == val:
            return
        self._driver_led_current = val
        state = self._virtual_read(_AS726X_LED_CONTROL)
        state &= ~(0x3 << 4)
        state = state | (Adafruit_AS726x.DRIVER_CURRENT_LIMITS.index(val) << 4)
        self._virtual_write(_AS726X_LED_CONTROL, state)

    @property
    def indicator_led_current(self):
        """The current limit for the indicator LED in milliamps. One of:

           - 1 mA
           - 2 mA
           - 4 mA
           - 8 mA"""
        return self._indicator_led_current

    @indicator_led_current.setter
    def indicator_led_current(self, val):
        if val not in Adafruit_AS726x.INDICATOR_CURRENT_LIMITS:
            raise ValueError("Must be 1, 2, 4 or 8")
        if self._indicator_led_current == val:
            return
        self._indicator_led_current = val
        state = self._virtual_read(_AS726X_LED_CONTROL)
        state &= ~(0x3 << 1)
        state = state | (Adafruit_AS726x.INDICATOR_CURRENT_LIMITS.index(val) << 4)
        self._virtual_write(_AS726X_LED_CONTROL, state)

    @property
    def conversion_mode(self):
        """The conversion mode. One of:

           - `MODE_0`
           - `MODE_1`
           - `MODE_2`
           - `ONE_SHOT`"""
        return self._conversion_mode

    @conversion_mode.setter
    def conversion_mode(self, val):
        val = int(val)
        assert self.MODE_0 <= val <= self.ONE_SHOT
        if self._conversion_mode == val:
            return
        self._conversion_mode = val
        state = self._virtual_read(_AS726X_CONTROL_SETUP)
        state &= ~(val << 2)
        self._virtual_write(_AS726X_CONTROL_SETUP, state | (val << 2))

    @property
    def gain(self):
        """The gain for the sensor"""
        return self._gain

    @gain.setter
    def gain(self, val):
        if val not in Adafruit_AS726x.GAIN:
            raise ValueError("Must be 1, 3.7, 16 or 64")
        if self._gain == val:
            return
        self._gain = val
        state = self._virtual_read(_AS726X_CONTROL_SETUP)
        state &= ~(0x3 << 4)
        state |= (Adafruit_AS726x.GAIN.index(val) << 4)
        self._virtual_write(_AS726X_CONTROL_SETUP, state)

    @property
    def integration_time(self):
        """The integration time in milliseconds between 2.8 and 714 ms"""
        return self._integration_time

    @integration_time.setter
    def integration_time(self, val):
        val = int(val)
        if not 2.8 <= val <= 714:
            raise ValueError("Out of supported range 2.8 - 714 ms")
        if self._integration_time == val:
            return
        self._integration_time = val
        self._virtual_write(_AS726X_INT_T, int(val/2.8))

    def start_measurement(self):
        """Begin a measurement.

           This will set the device to One Shot mode and values will not change after `data_ready`
           until `start_measurement` is called again or the `conversion_mode` is changed."""
        state = self._virtual_read(_AS726X_CONTROL_SETUP)
        state &= ~(0x02)
        self._virtual_write(_AS726X_CONTROL_SETUP, state)

        self.conversion_mode = self.ONE_SHOT

    def read_channel(self, channel):
        """Read an individual sensor channel"""
        return (self._virtual_read(channel) << 8) | self._virtual_read(channel + 1)

    def read_calibrated_value(self, channel):
        """Read a calibrated sensor channel"""
        val = bytearray(4)
        val[0] = self._virtual_read(channel)
        val[1] = self._virtual_read(channel + 1)
        val[2] = self._virtual_read(channel + 2)
        val[3] = self._virtual_read(channel + 3)
        return struct.unpack('!f', val)[0]

    @property
    def data_ready(self):
        """True if the sensor has data ready to be read, False otherwise"""
        return (self._virtual_read(_AS726X_CONTROL_SETUP) >> 1) & 0x01

    @property
    def temperature(self):
        """The temperature of the device in Celsius"""
        return self._virtual_read(_AS726X_DEVICE_TEMP)

    def _read_u8(self, command):
        """read a single byte from a specified register"""
        buf = self.buf2
        buf[0] = command
        with self.i2c_device as i2c:
            i2c.write(buf, end=1)
            i2c.readinto(buf, end=1)
        return buf[0]

    def __write_u8(self, command, abyte):
        """Write a command and 1 byte of data to the I2C device"""
        buf = self.buf2
        buf[0] = command
        buf[1] = abyte
        with self.i2c_device as i2c:
            i2c.write(buf)

    def _virtual_read(self, addr):
        """read a virtual register"""
        while True:
            # Read slave I2C status to see if the read buffer is ready.
            status = self._read_u8(_AS726X_SLAVE_STATUS_REG)
            if (status & _AS726X_SLAVE_TX_VALID) == 0:
                # No inbound TX pending at slave. Okay to write now.
                break
        # Send the virtual register address (setting bit 7 to indicate a pending write).
        self.__write_u8(_AS726X_SLAVE_WRITE_REG, addr)
        while True:
            # Read the slave I2C status to see if our read data is available.
            status = self._read_u8(_AS726X_SLAVE_STATUS_REG)
            if (status & _AS726X_SLAVE_RX_VALID) != 0:
                # Read data is ready.
                break
        # Read the data to complete the operation.
        data = self._read_u8(_AS726X_SLAVE_READ_REG)
        return data

    def _virtual_write(self, addr, value):
        """write a virtual register"""
        while True:
            # Read slave I2C status to see if the write buffer is ready.
            status = self._read_u8(_AS726X_SLAVE_STATUS_REG)
            if (status & _AS726X_SLAVE_TX_VALID) == 0:
                break # No inbound TX pending at slave. Okay to write now.
        # Send the virtual register address (setting bit 7 to indicate a pending write).
        self.__write_u8(_AS726X_SLAVE_WRITE_REG, (addr | 0x80))
        while True:
            # Read the slave I2C status to see if the write buffer is ready.
            status = self._read_u8(_AS726X_SLAVE_STATUS_REG)
            if (status & _AS726X_SLAVE_TX_VALID) == 0:
                break # No inbound TX pending at slave. Okay to write data now.

        # Send the data to complete the operation.
        self.__write_u8(_AS726X_SLAVE_WRITE_REG, value)

class AS7262(Adafruit_AS726x):
    def __init__(self, i2c_bus):
        super().__init__(i2c_bus)
        self.integration_time = 140
    
    @property
    def violet(self):
        """Calibrated violet (450nm) value"""
        return self.read_calibrated_value(_AS7262_VIOLET_CALIBRATED)

    @property
    def blue(self):
        """Calibrated blue (500nm) value"""
        return self.read_calibrated_value(_AS7262_BLUE_CALIBRATED)

    @property
    def green(self):
        """Calibrated green (550nm) value"""
        return self.read_calibrated_value(_AS7262_GREEN_CALIBRATED)

    @property
    def yellow(self):
        """Calibrated yellow (570nm) value"""
        return self.read_calibrated_value(_AS7262_YELLOW_CALIBRATED)

    @property
    def orange(self):
        """Calibrated orange (600nm) value"""
        return self.read_calibrated_value(_AS7262_ORANGE_CALIBRATED)

    @property
    def red(self):
        """Calibrated red (650nm) value"""
        return self.read_calibrated_value(_AS7262_RED_CALIBRATED)

    @property
    def raw_violet(self):
        """Raw violet (450nm) 16-bit value"""
        return self.read_channel(_AS7262_VIOLET)

    @property
    def raw_blue(self):
        """Raw blue (500nm) 16-bit value"""
        return self.read_channel(_AS7262_BLUE)

    @property
    def raw_green(self):
        """Raw green (550nm) 16-bit value"""
        return self.read_channel(_AS7262_GREEN)

    @property
    def raw_yellow(self):
        """Raw yellow (570nm) 16-bit value"""
        return self.read_channel(_AS7262_YELLOW)

    @property
    def raw_orange(self):
        """Raw orange (600nm) 16-bit value"""
        return self.read_channel(_AS7262_ORANGE)

    @property
    def raw_red(self):
        """Raw red (650nm) 16-bit value"""
        return self.read_channel(_AS7262_RED)

class AS7263(Adafruit_AS726x):
    def __init__(self, i2c_bus):
        super().__init__(i2c_bus)
        self.integration_time = 280
    
    @property
    def raw_nir_r(self):
        """NIR Raw R"""
        return self.read_channel(_AS7263_R)

    @property
    def raw_nir_s(self):
        """NIR Raw S"""
        return self.read_channel(_AS7263_S)
    
    @property
    def raw_nir_t(self):
        """NIR Raw T"""
        return self.read_channel(_AS7263_T)
    
    @property
    def raw_nir_u(self):
        """NIR Raw U"""
        return self.read_channel(_AS7263_U)
    
    @property
    def raw_nir_v(self):
        """NIR Raw V"""
        return self.read_channel(_AS7263_V)

    @property
    def raw_nir_w(self):
        """NIR Raw W"""
        return self.read_channel(_AS7263_W)
    
    @property
    def nir_r(self):
        """NIR Calibrated R"""
        return self.read_calibrated_value(_AS7263_R_CAL)
    
    @property
    def nir_s(self):
        """NIR Calibrated S"""
        return self.read_calibrated_value(_AS7263_S_CAL)
    
    @property
    def nir_t(self):
        """NIR Calibrated T"""
        return self.read_calibrated_value(_AS7263_T_CAL)

    @property
    def nir_u(self):
        """NIR Calibrated U"""
        return self.read_calibrated_value(_AS7263_U_CAL)
    
    @property
    def nir_v(self):
        """NIR Calibrated V"""
        return self.read_calibrated_value(_AS7263_V_CAL)

    @property
    def nir_w(self):
        """NIR Calibrated W"""
        return self.read_calibrated_value(_AS7263_W_CAL)

class AS7261(Adafruit_AS726x):

    def __init__(self, i2c_bus):
        super().__init__(i2c_bus)
        self.integration_time = 400
    
    @property
    def raw_x_high(self):
        """Channel X High Data Byte"""
        return self.read_channel(_AS7261_X_HIGH)
    
    @property
    def raw_x_low(self):
        """Channel X Low Data Byte"""
        return self.read_channel(_AS7261_X_LOW)
    
    @property
    def raw_y_high(self):
        """Channel Y High Data Byte"""
        return self.read_channel(_AS7261_Y_HIGH)
   
    @property
    def raw_y_low(self):
        """Channel Y Low Data Byte"""
        return self.read_channel(_AS7261_Y_LOW)
    
    @property
    def raw_z_high(self):
        """Channel Z High Data Byte"""
        return self.read_channel(_AS7261_Z_HIGH)

    @property
    def raw_z_low(self):
        """Channel Z Low Data Byte"""
        return self.read_channel(_AS7261_Z_LOW)

    @property
    def nir_high(self):
        """Channel NIR High"""
        return self.read_channel(_AS7261_NIR_HIGH)

    @property
    def nir_low(self):
        """Channel NIR Low"""
        return self.read_channel(_AS7261_NIR_LOW)

    @property
    def dark_high(self):
        """Channel DARK High"""
        return self.read_channel(_AS7261_DARK_HIGH)

    @property
    def dark_low(self):
        """Channel DARK Low"""
        return self.read_channel(_AS7261_DARK_LOW)

    @property
    def clear_high(self):
        """Channel Clear High"""
        return self.read_channel(_AS7261_CLEAR_HIGH)

    @property
    def clear_low(self):
        """Channel Clear Low"""
        return self.read_channel(_AS7261_CLEAR_LOW)

    @property
    def cal_x(self):
        """Channel Calibrated X"""
        return self.read_channel(_AS7261_CAL_X)

    @property
    def cal_y(self):
        """Channel Calibrated Y"""
        return self.read_channel(_AS7261_CAL_Y)

    @property
    def cal_z(self):
        """Channel Calibrated Z"""
        return self.read_channel(_AS7261_CAL_Z)

    @property
    def cal_x_1931(self):
        """Channel Calibrated X (CIE 1931)"""
        return self.read_channel(_AS7261_CAL_X_1931)

    @property
    def cal_y_1931(self):
        """Channel Calibrated Y (CIE 1931)"""
        return self.read_channel(_AS7261_CAL_Y_1931)

    @property
    def cal_upri(self):
        """Channel Calibrated u' (CIE 1976)"""
        return self.read_channel(_AS7261_CAL_UPRI)

    @property
    def cal_vpri(self):
        """Channel Calibrated v' (CIE 1976)"""
        return self.read_channel(_AS7261_CAL_VPRI)

    @property
    def cal_u(self):
        """Channel Calibrated u (CIE 1976)"""
        return self.read_channel(_AS7261_CAL_U)

    @property
    def cal_v(self):
        """Channel Calibrated v (CIE 1976)"""
        return self.read_channel(_AS7261_CAL_V)

    @property
    def cal_duv(self):
        """Channel Calibrated DUV (CIE 1976)"""
        return self.read_channel(_AS7261_CAL_DUV)

    @property
    def cal_lux(self):
        """Channel Calibrated LUX"""
        return self.read_channel(_AS7261_CAL_LUX)

    @property
    def cal_cct(self):
        """Channel Calibrated CCT"""
        return self.read_channel(_AS7261_CAL_CCT)


#pylint: enable=too-many-instance-attributes
#pylint: enable=too-many-public-methods