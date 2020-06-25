# combined files from python distribution, to debug Adafruit battery read issue

### smbus.py

# Copyright (c) 2016 Adafruit Industries
# Author: Tony DiCola
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
"""I2C interface that mimics the Python SMBus API."""

from ctypes import c_uint8, c_uint16, c_uint32, cast, pointer, POINTER
from ctypes import create_string_buffer, Structure
from fcntl import ioctl
import struct

# I2C C API constants (from linux kernel headers)
# pylint: disable=bad-whitespace
I2C_M_TEN             = 0x0010  # this is a ten bit chip address
I2C_M_RD              = 0x0001  # read data, from slave to master
I2C_M_STOP            = 0x8000  # if I2C_FUNC_PROTOCOL_MANGLING
I2C_M_NOSTART         = 0x4000  # if I2C_FUNC_NOSTART
I2C_M_REV_DIR_ADDR    = 0x2000  # if I2C_FUNC_PROTOCOL_MANGLING
I2C_M_IGNORE_NAK      = 0x1000  # if I2C_FUNC_PROTOCOL_MANGLING
I2C_M_NO_RD_ACK       = 0x0800  # if I2C_FUNC_PROTOCOL_MANGLING
I2C_M_RECV_LEN        = 0x0400  # length will be first received byte

I2C_SLAVE             = 0x0703  # Use this slave address
I2C_SLAVE_FORCE       = 0x0706  # Use this slave address, even if
                                # is already in use by a driver!
I2C_TENBIT            = 0x0704  # 0 for 7 bit addrs, != 0 for 10 bit
I2C_FUNCS             = 0x0705  # Get the adapter functionality mask
I2C_RDWR              = 0x0707  # Combined R/W transfer (one STOP only)
I2C_PEC               = 0x0708  # != 0 to use PEC with SMBus
I2C_SMBUS             = 0x0720  # SMBus transfer
# pylint: enable=bad-whitespace


# ctypes versions of I2C structs defined by kernel.
# Tone down pylint for the Python classes that mirror C structs.
#pylint: disable=invalid-name,too-few-public-methods
class i2c_msg(Structure):
    """Linux i2c_msg struct."""
    _fields_ = [
        ('addr', c_uint16),
        ('flags', c_uint16),
        ('len', c_uint16),
        ('buf', POINTER(c_uint8))
    ]

class i2c_rdwr_ioctl_data(Structure): #pylint: disable=invalid-name
    """Linux i2c data struct."""
    _fields_ = [
        ('msgs', POINTER(i2c_msg)),
        ('nmsgs', c_uint32)
    ]
#pylint: enable=invalid-name,too-few-public-methods

def make_i2c_rdwr_data(messages):
    """Utility function to create and return an i2c_rdwr_ioctl_data structure
    populated with a list of specified I2C messages.  The messages parameter
    should be a list of tuples which represent the individual I2C messages to
    send in this transaction.  Tuples should contain 4 elements: address value,
    flags value, buffer length, ctypes c_uint8 pointer to buffer.
    """
    # Create message array and populate with provided data.
    msg_data_type = i2c_msg*len(messages)
    msg_data = msg_data_type()
    for i, message in enumerate(messages):
        msg_data[i].addr = message[0] & 0x7F
        msg_data[i].flags = message[1]
        msg_data[i].len = message[2]
        msg_data[i].buf = message[3]
    # Now build the data structure.
    data = i2c_rdwr_ioctl_data()
    data.msgs = msg_data
    data.nmsgs = len(messages)
    return data

# Create an interface that mimics the Python SMBus API.
class SMBus(object):
    """I2C interface that mimics the Python SMBus API but is implemented with
    pure Python calls to ioctl and direct /dev/i2c device access.
    """

    def __init__(self, bus=None):
        """Create a new smbus instance.  Bus is an optional parameter that
        specifies the I2C bus number to use, for example 1 would use device
        /dev/i2c-1.  If bus is not specified then the open function should be
        called to open the bus.
        """
        self._device = None
        if bus is not None:
            self.open(bus)

    def __del__(self):
        """Clean up any resources used by the SMBus instance."""
        self.close()

    def __enter__(self):
        """Context manager enter function."""
        # Just return this object so it can be used in a with statement, like
        # with SMBus(1) as bus:
        #     # do stuff!
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit function, ensures resources are cleaned up."""
        self.close()
        return False  # Don't suppress exceptions.

    def open(self, bus):
        """Open the smbus interface on the specified bus."""
        # Close the device if it's already open.
        if self._device is not None:
            self.close()
        # Try to open the file for the specified bus.  Must turn off buffering
        # or else Python 3 fails (see: https://bugs.python.org/issue20074)
        self._device = open('/dev/i2c-{0}'.format(bus), 'r+b', buffering=0)
        # TODO: Catch IOError and throw a better error message that describes
        # what's wrong (i.e. I2C may not be enabled or the bus doesn't exist).

    def close(self):
        """Close the smbus connection.  You cannot make any other function
        calls on the bus unless open is called!"""
        if self._device is not None:
            self._device.close()
            self._device = None

    def _select_device(self, addr):
        """Set the address of the device to communicate with on the I2C bus."""
        ioctl(self._device.fileno(), I2C_SLAVE, addr & 0x7F)

    def read_byte(self, addr):
        """Read a single byte from the specified device."""
        assert self._device is not None, 'Bus must be opened before operations are made against it!'
        self._select_device(addr)
        return ord(self._device.read(1))

    def read_bytes(self, addr, number):
        """Read many bytes from the specified device."""
        assert self._device is not None, 'Bus must be opened before operations are made against it!'
        self._select_device(addr)
        return self._device.read(number)

    def read_byte_data(self, addr, cmd):
        """Read a single byte from the specified cmd register of the device."""
        assert self._device is not None, 'Bus must be opened before operations are made against it!'
        # Build ctypes values to marshall between ioctl and Python.
        reg = c_uint8(cmd)
        result = c_uint8()
        # Build ioctl request.
        request = make_i2c_rdwr_data([
            (addr, 0, 1, pointer(reg)),             # Write cmd register.
            (addr, I2C_M_RD, 1, pointer(result))    # Read 1 byte as result.
        ])
        # Make ioctl call and return result data.
        ioctl(self._device.fileno(), I2C_RDWR, request)
        return result.value

    def read_word_data(self, addr, cmd):
        """Read a word (2 bytes) from the specified cmd register of the device.
        Note that this will interpret data using the endianness of the processor
        running Python (typically little endian)!
        """
        assert self._device is not None, 'Bus must be opened before operations are made against it!'
        # Build ctypes values to marshall between ioctl and Python.
        reg = c_uint8(cmd)
        result = c_uint16()
        # Build ioctl request.
        request = make_i2c_rdwr_data([
            (addr, 0, 1, pointer(reg)),             # Write cmd register.
            (addr, I2C_M_RD, 2, cast(pointer(result), POINTER(c_uint8)))   # Read word (2 bytes).
        ])
        # Make ioctl call and return result data.
        ioctl(self._device.fileno(), I2C_RDWR, request)
        return result.value

    def read_block_data(self, addr, cmd):
        """Perform a block read from the specified cmd register of the device.
        The amount of data read is determined by the first byte send back by
        the device.  Data is returned as a bytearray.
        """
        # TODO: Unfortunately this will require calling the low level I2C
        # access ioctl to trigger a proper read_block_data.  The amount of data
        # returned isn't known until the device starts responding so an I2C_RDWR
        # ioctl won't work.
        raise NotImplementedError()

    def read_i2c_block_data(self, addr, cmd, length=32):
        """Perform a read from the specified cmd register of device.  Length number
        of bytes (default of 32) will be read and returned as a bytearray.
        """
        assert self._device is not None, 'Bus must be opened before operations are made against it!'
        # Build ctypes values to marshall between ioctl and Python.

        # convert register into bytearray
        if not isinstance(cmd, (bytes, bytearray)):
            reg = cmd  # backup
            print('cmd is {}'.format(cmd))
            cmd = bytearray(1)
            cmd[0] = reg

        cmdstring = create_string_buffer(len(cmd))
        for i, val in enumerate(cmd):
            print('i: {} val: {} type(val): {}'.format(i, val, type(val)))
            cmdstring[i] = val

        result = create_string_buffer(length)

        # Build ioctl request.
        request = make_i2c_rdwr_data([
            (addr, 0, len(cmd), cast(cmdstring, POINTER(c_uint8))),    # Write cmd register.
            (addr, I2C_M_RD, length, cast(result, POINTER(c_uint8)))   # Read data.
            ])

        # Make ioctl call and return result data.
        ioctl(self._device.fileno(), I2C_RDWR, request)
        return bytearray(result.raw)  # Use .raw instead of .value which will stop at a null byte!

    def write_quick(self, addr):
        """Write a single byte to the specified device."""
        # What a strange function, from the python-smbus source this appears to
        # just write a single byte that initiates a write to the specified device
        # address (but writes no data!).  The functionality is duplicated below
        # but the actual use case for this is unknown.
        assert self._device is not None, 'Bus must be opened before operations are made against it!'
        # Build ioctl request.
        request = make_i2c_rdwr_data([
            (addr, 0, 0, None),  # Write with no data.
        ])
        # Make ioctl call and return result data.
        ioctl(self._device.fileno(), I2C_RDWR, request)

    def write_byte(self, addr, val):
        """Write a single byte to the specified device."""
        assert self._device is not None, 'Bus must be opened before operations are made against it!'
        self._select_device(addr)
        data = bytearray(1)
        data[0] = val & 0xFF
        self._device.write(data)

    def write_bytes(self, addr, buf):
        """Write many bytes to the specified device. buf is a bytearray"""
        assert self._device is not None, 'Bus must be opened before operations are made against it!'
        self._select_device(addr)
        self._device.write(buf)

    def write_byte_data(self, addr, cmd, val):
        """Write a byte of data to the specified cmd register of the device.
        """
        assert self._device is not None, 'Bus must be opened before operations are made against it!'
        # Construct a string of data to send with the command register and byte value.
        data = bytearray(2)
        data[0] = cmd & 0xFF
        data[1] = val & 0xFF
        # Send the data to the device.
        self._select_device(addr)
        self._device.write(data)

    def write_word_data(self, addr, cmd, val):
        """Write a word (2 bytes) of data to the specified cmd register of the
        device.  Note that this will write the data in the endianness of the
        processor running Python (typically little endian)!
        """
        assert self._device is not None, 'Bus must be opened before operations are made against it!'
        # Construct a string of data to send with the command register and word value.
        data = struct.pack('=BH', cmd & 0xFF, val & 0xFFFF)
        # Send the data to the device.
        self._select_device(addr)
        self._device.write(data)

    def write_block_data(self, addr, cmd, vals):
        """Write a block of data to the specified cmd register of the device.
        The amount of data to write should be the first byte inside the vals
        string/bytearray and that count of bytes of data to write should follow
        it.
        """
        # Just use the I2C block data write to write the provided values and
        # their length as the first byte.
        data = bytearray(len(vals)+1)
        data[0] = len(vals) & 0xFF
        data[1:] = vals[0:]
        self.write_i2c_block_data(addr, cmd, data)

    def write_i2c_block_data(self, addr, cmd, vals):
        """Write a buffer of data to the specified cmd register of the device.
        """
        assert self._device is not None, 'Bus must be opened before operations are made against it!'
        # Construct a string of data to send, including room for the command register.
        data = bytearray(len(vals)+1)
        data[0] = cmd & 0xFF  # Command register at the start.
        data[1:] = vals[0:]   # Copy in the block data (ugly but necessary to ensure
                              # the entire write happens in one transaction).
        # Send the data to the device.
        self._select_device(addr)
        self._device.write(data)

    def process_call(self, addr, cmd, val):
        """Perform a smbus process call by writing a word (2 byte) value to
        the specified register of the device, and then reading a word of response
        data (which is returned).
        """
        assert self._device is not None, 'Bus must be opened before operations are made against it!'
        # Build ctypes values to marshall between ioctl and Python.
        data = create_string_buffer(struct.pack('=BH', cmd, val))
        result = c_uint16()
        # Build ioctl request.
        request = make_i2c_rdwr_data([
            (addr, 0, 3, cast(pointer(data), POINTER(c_uint8))),          # Write data.
            (addr, I2C_M_RD, 2, cast(pointer(result), POINTER(c_uint8)))  # Read word (2 bytes).
        ])
        # Make ioctl call and return result data.
        ioctl(self._device.fileno(), I2C_RDWR, request)
        # Note the python-smbus code appears to have a rather serious bug and
        # does not return the result value!  This is fixed below by returning it.
        return result.value

### I2c.py

# Copyright (c) 2014 Adafruit Industries
# Author: Tony DiCola
# Based on Adafruit_I2C.py created by Kevin Townsend.
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
import logging
import subprocess

import Adafruit_GPIO.Platform as Platform


def reverseByteOrder(data):
    """Reverses the byte order of an int (16-bit) or long (32-bit) value."""
    # Courtesy Vishal Sapre
    byteCount = len(hex(data)[2:].replace('L','')[::2])
    val       = 0
    for i in range(byteCount):
        val    = (val << 8) | (data & 0xff)
        data >>= 8
    return val

def get_default_bus():
    """Return the default bus number based on the device platform.  For a
    Raspberry Pi either bus 0 or 1 (based on the Pi revision) will be returned.
    For a Beaglebone Black the first user accessible bus, 1, will be returned.
    """
    plat = Platform.platform_detect()
    if plat == Platform.RASPBERRY_PI:
        if Platform.pi_revision() == 1:
            # Revision 1 Pi uses I2C bus 0.
            return 0
        else:
            # Revision 2 Pi uses I2C bus 1.
            return 1
    elif plat == Platform.BEAGLEBONE_BLACK:
        # Beaglebone Black has multiple I2C buses, default to 1 (P9_19 and P9_20).
        return 1
    else:
        raise RuntimeError('Could not determine default I2C bus for platform.')

def get_i2c_device(address, busnum=None, i2c_interface=None, **kwargs):
    """Return an I2C device for the specified address and on the specified bus.
    If busnum isn't specified, the default I2C bus for the platform will attempt
    to be detected.
    """
    if busnum is None:
        busnum = get_default_bus()
    return Device(address, busnum, i2c_interface, **kwargs)

def require_repeated_start():
    """Enable repeated start conditions for I2C register reads.  This is the
    normal behavior for I2C, however on some platforms like the Raspberry Pi
    there are bugs which disable repeated starts unless explicitly enabled with
    this function.  See this thread for more details:
      http://www.raspberrypi.org/forums/viewtopic.php?f=44&t=15840
    """
    plat = Platform.platform_detect()
    if plat == Platform.RASPBERRY_PI:
        # On the Raspberry Pi there is a bug where register reads don't send a
        # repeated start condition like the kernel smbus I2C driver functions
        # define.  As a workaround this bit in the BCM2708 driver sysfs tree can
        # be changed to enable I2C repeated starts.
        subprocess.check_call('chmod 666 /sys/module/i2c_bcm2708/parameters/combined', shell=True)
        subprocess.check_call('echo -n 1 > /sys/module/i2c_bcm2708/parameters/combined', shell=True)
    # Other platforms are a no-op because they (presumably) have the correct
    # behavior and send repeated starts.


class Device(object):
    """Class for communicating with an I2C device using the adafruit-pureio pure
    python smbus library, or other smbus compatible I2C interface. Allows reading
    and writing 8-bit, 16-bit, and byte array values to registers
    on the device."""
    def __init__(self, address, busnum, i2c_interface=None):
        """Create an instance of the I2C device at the specified address on the
        specified I2C bus number."""
        self._address = address
        if i2c_interface is None:
            # Use pure python I2C interface if none is specified.
            # import Adafruit_PureIO.smbus
            #self._bus = Adafruit_PureIO.smbus.SMBus(busnum)
            self._bus = SMBus(busnum)
        else:
            # Otherwise use the provided class to create an smbus interface.
            self._bus = i2c_interface(busnum)
        self._logger = logging.getLogger('Adafruit_I2C.Device.Bus.{0}.Address.{1:#0X}' \
                                .format(busnum, address))

    def writeRaw8(self, value):
        """Write an 8-bit value on the bus (without register)."""
        value = value & 0xFF
        self._bus.write_byte(self._address, value)
        self._logger.debug("Wrote 0x%02X",
                     value)

    def write8(self, register, value):
        """Write an 8-bit value to the specified register."""
        value = value & 0xFF
        self._bus.write_byte_data(self._address, register, value)
        self._logger.debug("Wrote 0x%02X to register 0x%02X",
                     value, register)

    def write16(self, register, value):
        """Write a 16-bit value to the specified register."""
        value = value & 0xFFFF
        self._bus.write_word_data(self._address, register, value)
        self._logger.debug("Wrote 0x%04X to register pair 0x%02X, 0x%02X",
                     value, register, register+1)

    def writeList(self, register, data):
        """Write bytes to the specified register."""
        self._bus.write_i2c_block_data(self._address, register, data)
        self._logger.debug("Wrote to register 0x%02X: %s",
                     register, data)

    def readList(self, register, length):
        """Read a length number of bytes from the specified register.  Results
        will be returned as a bytearray."""
        print('_address: {} register: {}'.format(self._address, register))
        results = self._bus.read_i2c_block_data(self._address, register, length)
        self._logger.debug("Read the following from register 0x%02X: %s",
                     register, results)
        return results

    def readRaw8(self):
        """Read an 8-bit value on the bus (without register)."""
        result = self._bus.read_byte(self._address) & 0xFF
        self._logger.debug("Read 0x%02X",
                    result)
        return result

    def readU8(self, register):
        """Read an unsigned byte from the specified register."""
        result = self._bus.read_byte_data(self._address, register) & 0xFF
        self._logger.debug("Read 0x%02X from register 0x%02X",
                     result, register)
        return result

    def readS8(self, register):
        """Read a signed byte from the specified register."""
        result = self.readU8(register)
        if result > 127:
            result -= 256
        return result

    def readU16(self, register, little_endian=True):
        """Read an unsigned 16-bit value from the specified register, with the
        specified endianness (default little endian, or least significant byte
        first)."""
        result = self._bus.read_word_data(self._address,register) & 0xFFFF
        self._logger.debug("Read 0x%04X from register pair 0x%02X, 0x%02X",
                           result, register, register+1)
        # Swap bytes if using big endian because read_word_data assumes little
        # endian on ARM (little endian) systems.
        if not little_endian:
            result = ((result << 8) & 0xFF00) + (result >> 8)
        return result

    def readS16(self, register, little_endian=True):
        """Read a signed 16-bit value from the specified register, with the
        specified endianness (default little endian, or least significant byte
        first)."""
        result = self.readU16(register, little_endian)
        if result > 32767:
            result -= 65536
        return result

    def readU16LE(self, register):
        """Read an unsigned 16-bit value from the specified register, in little
        endian byte order."""
        return self.readU16(register, little_endian=True)

    def readU16BE(self, register):
        """Read an unsigned 16-bit value from the specified register, in big
        endian byte order."""
        return self.readU16(register, little_endian=False)

    def readS16LE(self, register):
        """Read a signed 16-bit value from the specified register, in little
        endian byte order."""
        return self.readS16(register, little_endian=True)

    def readS16BE(self, register):
        """Read a signed 16-bit value from the specified register, in big
        endian byte order."""
        return self.readS16(register, little_endian=False)

### ADS1x15.py
# Copyright (c) 2016 Adafruit Industries
# Author: Tony DiCola
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
import time


# Register and other configuration values:
ADS1x15_DEFAULT_ADDRESS        = 0x48
ADS1x15_POINTER_CONVERSION     = 0x00
ADS1x15_POINTER_CONFIG         = 0x01
ADS1x15_POINTER_LOW_THRESHOLD  = 0x02
ADS1x15_POINTER_HIGH_THRESHOLD = 0x03
ADS1x15_CONFIG_OS_SINGLE       = 0x8000
ADS1x15_CONFIG_MUX_OFFSET      = 12
# Maping of gain values to config register values.
ADS1x15_CONFIG_GAIN = {
    2/3: 0x0000,
    1:   0x0200,
    2:   0x0400,
    4:   0x0600,
    8:   0x0800,
    16:  0x0A00
}
ADS1x15_CONFIG_MODE_CONTINUOUS  = 0x0000
ADS1x15_CONFIG_MODE_SINGLE      = 0x0100
# Mapping of data/sample rate to config register values for ADS1015 (faster).
ADS1015_CONFIG_DR = {
    128:   0x0000,
    250:   0x0020,
    490:   0x0040,
    920:   0x0060,
    1600:  0x0080,
    2400:  0x00A0,
    3300:  0x00C0
}
# Mapping of data/sample rate to config register values for ADS1115 (slower).
ADS1115_CONFIG_DR = {
    8:    0x0000,
    16:   0x0020,
    32:   0x0040,
    64:   0x0060,
    128:  0x0080,
    250:  0x00A0,
    475:  0x00C0,
    860:  0x00E0
}
ADS1x15_CONFIG_COMP_WINDOW      = 0x0010
ADS1x15_CONFIG_COMP_ACTIVE_HIGH = 0x0008
ADS1x15_CONFIG_COMP_LATCHING    = 0x0004
ADS1x15_CONFIG_COMP_QUE = {
    1: 0x0000,
    2: 0x0001,
    4: 0x0002
}
ADS1x15_CONFIG_COMP_QUE_DISABLE = 0x0003


class ADS1x15(object):
    """Base functionality for ADS1x15 analog to digital converters."""

    def __init__(self, address=ADS1x15_DEFAULT_ADDRESS, i2c=None, **kwargs):
        #if i2c is None:
            #import Adafruit_GPIO.I2C as I2C
            #i2c = I2C
        #self._device = i2c.get_i2c_device(address, **kwargs)
        self._device = get_i2c_device(address, **kwargs)

    def _data_rate_default(self):
        """Retrieve the default data rate for this ADC (in samples per second).
        Should be implemented by subclasses.
        """
        raise NotImplementedError('Subclasses must implement _data_rate_default!')

    def _data_rate_config(self, data_rate):
        """Subclasses should override this function and return a 16-bit value
        that can be OR'ed with the config register to set the specified
        data rate.  If a value of None is specified then a default data_rate
        setting should be returned.  If an invalid or unsupported data_rate is
        provided then an exception should be thrown.
        """
        raise NotImplementedError('Subclass must implement _data_rate_config function!')

    def _conversion_value(self, low, high):
        """Subclasses should override this function that takes the low and high
        byte of a conversion result and returns a signed integer value.
        """
        raise NotImplementedError('Subclass must implement _conversion_value function!')

    def _read(self, mux, gain, data_rate, mode):
        """Perform an ADC read with the provided mux, gain, data_rate, and mode
        values.  Returns the signed integer result of the read.
        """
        config = ADS1x15_CONFIG_OS_SINGLE  # Go out of power-down mode for conversion.
        # Specify mux value.
        config |= (mux & 0x07) << ADS1x15_CONFIG_MUX_OFFSET
        # Validate the passed in gain and then set it in the config.
        if gain not in ADS1x15_CONFIG_GAIN:
            raise ValueError('Gain must be one of: 2/3, 1, 2, 4, 8, 16')
        config |= ADS1x15_CONFIG_GAIN[gain]
        # Set the mode (continuous or single shot).
        config |= mode
        # Get the default data rate if none is specified (default differs between
        # ADS1015 and ADS1115).
        if data_rate is None:
            data_rate = self._data_rate_default()
        # Set the data rate (this is controlled by the subclass as it differs
        # between ADS1015 and ADS1115).
        config |= self._data_rate_config(data_rate)
        config |= ADS1x15_CONFIG_COMP_QUE_DISABLE  # Disble comparator mode.
        # Send the config value to start the ADC conversion.
        # Explicitly break the 16-bit value down to a big endian pair of bytes.
        self._device.writeList(ADS1x15_POINTER_CONFIG, [(config >> 8) & 0xFF, config & 0xFF])
        # Wait for the ADC sample to finish based on the sample rate plus a
        # small offset to be sure (0.1 millisecond).
        time.sleep(1.0/data_rate+0.0001)
        # Retrieve the result.
        result = self._device.readList(ADS1x15_POINTER_CONVERSION, 2)
        return self._conversion_value(result[1], result[0])

    def _read_comparator(self, mux, gain, data_rate, mode, high_threshold,
                         low_threshold, active_low, traditional, latching,
                         num_readings):
        """Perform an ADC read with the provided mux, gain, data_rate, and mode
        values and with the comparator enabled as specified.  Returns the signed
        integer result of the read.
        """
        assert num_readings == 1 or num_readings == 2 or num_readings == 4, 'Num readings must be 1, 2, or 4!'
        # Set high and low threshold register values.
        self._device.writeList(ADS1x15_POINTER_HIGH_THRESHOLD, [(high_threshold >> 8) & 0xFF, high_threshold & 0xFF])
        self._device.writeList(ADS1x15_POINTER_LOW_THRESHOLD, [(low_threshold >> 8) & 0xFF, low_threshold & 0xFF])
        # Now build up the appropriate config register value.
        config = ADS1x15_CONFIG_OS_SINGLE  # Go out of power-down mode for conversion.
        # Specify mux value.
        config |= (mux & 0x07) << ADS1x15_CONFIG_MUX_OFFSET
        # Validate the passed in gain and then set it in the config.
        if gain not in ADS1x15_CONFIG_GAIN:
            raise ValueError('Gain must be one of: 2/3, 1, 2, 4, 8, 16')
        config |= ADS1x15_CONFIG_GAIN[gain]
        # Set the mode (continuous or single shot).
        config |= mode
        # Get the default data rate if none is specified (default differs between
        # ADS1015 and ADS1115).
        if data_rate is None:
            data_rate = self._data_rate_default()
        # Set the data rate (this is controlled by the subclass as it differs
        # between ADS1015 and ADS1115).
        config |= self._data_rate_config(data_rate)
        # Enable window mode if required.
        if not traditional:
            config |= ADS1x15_CONFIG_COMP_WINDOW
        # Enable active high mode if required.
        if not active_low:
            config |= ADS1x15_CONFIG_COMP_ACTIVE_HIGH
        # Enable latching mode if required.
        if latching:
            config |= ADS1x15_CONFIG_COMP_LATCHING
        # Set number of comparator hits before alerting.
        config |= ADS1x15_CONFIG_COMP_QUE[num_readings]
        # Send the config value to start the ADC conversion.
        # Explicitly break the 16-bit value down to a big endian pair of bytes.
        self._device.writeList(ADS1x15_POINTER_CONFIG, [(config >> 8) & 0xFF, config & 0xFF])
        # Wait for the ADC sample to finish based on the sample rate plus a
        # small offset to be sure (0.1 millisecond).
        time.sleep(1.0/data_rate+0.0001)
        # Retrieve the result.
        result = self._device.readList(ADS1x15_POINTER_CONVERSION, 2)
        return self._conversion_value(result[1], result[0])

    def read_adc(self, channel, gain=1, data_rate=None):
        """Read a single ADC channel and return the ADC value as a signed integer
        result.  Channel must be a value within 0-3.
        """
        assert 0 <= channel <= 3, 'Channel must be a value within 0-3!'
        # Perform a single shot read and set the mux value to the channel plus
        # the highest bit (bit 3) set.
        return self._read(channel + 0x04, gain, data_rate, ADS1x15_CONFIG_MODE_SINGLE)

    def read_adc_difference(self, differential, gain=1, data_rate=None):
        """Read the difference between two ADC channels and return the ADC value
        as a signed integer result.  Differential must be one of:
          - 0 = Channel 0 minus channel 1
          - 1 = Channel 0 minus channel 3
          - 2 = Channel 1 minus channel 3
          - 3 = Channel 2 minus channel 3
        """
        assert 0 <= differential <= 3, 'Differential must be a value within 0-3!'
        # Perform a single shot read using the provided differential value
        # as the mux value (which will enable differential mode).
        return self._read(differential, gain, data_rate, ADS1x15_CONFIG_MODE_SINGLE)

    def start_adc(self, channel, gain=1, data_rate=None):
        """Start continuous ADC conversions on the specified channel (0-3). Will
        return an initial conversion result, then call the get_last_result()
        function to read the most recent conversion result. Call stop_adc() to
        stop conversions.
        """
        assert 0 <= channel <= 3, 'Channel must be a value within 0-3!'
        # Start continuous reads and set the mux value to the channel plus
        # the highest bit (bit 3) set.
        return self._read(channel + 0x04, gain, data_rate, ADS1x15_CONFIG_MODE_CONTINUOUS)

    def start_adc_difference(self, differential, gain=1, data_rate=None):
        """Start continuous ADC conversions between two ADC channels. Differential
        must be one of:
          - 0 = Channel 0 minus channel 1
          - 1 = Channel 0 minus channel 3
          - 2 = Channel 1 minus channel 3
          - 3 = Channel 2 minus channel 3
        Will return an initial conversion result, then call the get_last_result()
        function continuously to read the most recent conversion result.  Call
        stop_adc() to stop conversions.
        """
        assert 0 <= differential <= 3, 'Differential must be a value within 0-3!'
        # Perform a single shot read using the provided differential value
        # as the mux value (which will enable differential mode).
        return self._read(differential, gain, data_rate, ADS1x15_CONFIG_MODE_CONTINUOUS)

    def start_adc_comparator(self, channel, high_threshold, low_threshold,
                             gain=1, data_rate=None, active_low=True,
                             traditional=True, latching=False, num_readings=1):
        """Start continuous ADC conversions on the specified channel (0-3) with
        the comparator enabled.  When enabled the comparator to will check if
        the ADC value is within the high_threshold & low_threshold value (both
        should be signed 16-bit integers) and trigger the ALERT pin.  The
        behavior can be controlled by the following parameters:
          - active_low: Boolean that indicates if ALERT is pulled low or high
                        when active/triggered.  Default is true, active low.
          - traditional: Boolean that indicates if the comparator is in traditional
                         mode where it fires when the value is within the threshold,
                         or in window mode where it fires when the value is _outside_
                         the threshold range.  Default is true, traditional mode.
          - latching: Boolean that indicates if the alert should be held until
                      get_last_result() is called to read the value and clear
                      the alert.  Default is false, non-latching.
          - num_readings: The number of readings that match the comparator before
                          triggering the alert.  Can be 1, 2, or 4.  Default is 1.
        Will return an initial conversion result, then call the get_last_result()
        function continuously to read the most recent conversion result.  Call
        stop_adc() to stop conversions.
        """
        assert 0 <= channel <= 3, 'Channel must be a value within 0-3!'
        # Start continuous reads with comparator and set the mux value to the
        # channel plus the highest bit (bit 3) set.
        return self._read_comparator(channel + 0x04, gain, data_rate,
                                     ADS1x15_CONFIG_MODE_CONTINUOUS,
                                     high_threshold, low_threshold, active_low,
                                     traditional, latching, num_readings)

    def start_adc_difference_comparator(self, differential, high_threshold, low_threshold,
                                        gain=1, data_rate=None, active_low=True,
                                        traditional=True, latching=False, num_readings=1):
        """Start continuous ADC conversions between two channels with
        the comparator enabled.  See start_adc_difference for valid differential
        parameter values and their meaning.  When enabled the comparator to will
        check if the ADC value is within the high_threshold & low_threshold value
        (both should be signed 16-bit integers) and trigger the ALERT pin.  The
        behavior can be controlled by the following parameters:
          - active_low: Boolean that indicates if ALERT is pulled low or high
                        when active/triggered.  Default is true, active low.
          - traditional: Boolean that indicates if the comparator is in traditional
                         mode where it fires when the value is within the threshold,
                         or in window mode where it fires when the value is _outside_
                         the threshold range.  Default is true, traditional mode.
          - latching: Boolean that indicates if the alert should be held until
                      get_last_result() is called to read the value and clear
                      the alert.  Default is false, non-latching.
          - num_readings: The number of readings that match the comparator before
                          triggering the alert.  Can be 1, 2, or 4.  Default is 1.
        Will return an initial conversion result, then call the get_last_result()
        function continuously to read the most recent conversion result.  Call
        stop_adc() to stop conversions.
        """
        assert 0 <= differential <= 3, 'Differential must be a value within 0-3!'
        # Start continuous reads with comparator and set the mux value to the
        # channel plus the highest bit (bit 3) set.
        return self._read_comparator(differential, gain, data_rate,
                                     ADS1x15_CONFIG_MODE_CONTINUOUS,
                                     high_threshold, low_threshold, active_low,
                                     traditional, latching, num_readings)

    def stop_adc(self):
        """Stop all continuous ADC conversions (either normal or difference mode).
        """
        # Set the config register to its default value of 0x8583 to stop
        # continuous conversions.
        config = 0x8583
        self._device.writeList(ADS1x15_POINTER_CONFIG, [(config >> 8) & 0xFF, config & 0xFF])

    def get_last_result(self):
        """Read the last conversion result when in continuous conversion mode.
        Will return a signed integer value.
        """
        # Retrieve the conversion register value, convert to a signed int, and
        # return it.
        result = self._device.readList(ADS1x15_POINTER_CONVERSION, 2)
        return self._conversion_value(result[1], result[0])


class ADS1115(ADS1x15):
    """ADS1115 16-bit analog to digital converter instance."""

    def __init__(self, *args, **kwargs):
        super(ADS1115, self).__init__(*args, **kwargs)

    def _data_rate_default(self):
        # Default from datasheet page 16, config register DR bit default.
        return 128

    def _data_rate_config(self, data_rate):
        if data_rate not in ADS1115_CONFIG_DR:
            raise ValueError('Data rate must be one of: 8, 16, 32, 64, 128, 250, 475, 860')
        return ADS1115_CONFIG_DR[data_rate]

    def _conversion_value(self, low, high):
        # Convert to 16-bit signed value.
        value = ((high & 0xFF) << 8) | (low & 0xFF)
        # Check for sign bit and turn into a negative value if set.
        if value & 0x8000 != 0:
            value -= 1 << 16
        return value


class ADS1015(ADS1x15):
    """ADS1015 12-bit analog to digital converter instance."""

    def __init__(self, *args, **kwargs):
        super(ADS1015, self).__init__(*args, **kwargs)

    def _data_rate_default(self):
        # Default from datasheet page 19, config register DR bit default.
        return 1600

    def _data_rate_config(self, data_rate):
        if data_rate not in ADS1015_CONFIG_DR:
            raise ValueError('Data rate must be one of: 128, 250, 490, 920, 1600, 2400, 3300')
        return ADS1015_CONFIG_DR[data_rate]

    def _conversion_value(self, low, high):
        # Convert to 12-bit signed value.
        value = ((high & 0xFF) << 4) | ((low & 0xFF) >> 4)
        # Check for sign bit and turn into a negative value if set.
        if value & 0x800 != 0:
            value -= 1 << 12
        return value
