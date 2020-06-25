# method to read battery voltage. Had to modify some of the library calls to work with both python2 and python3
import sys
from Adafruit_ADS1x15 import ADS1115
from Adafruit_GPIO.I2C import Device

'''
Per ads1114 spec sheet, at gain=1 battery measures +- 4.096 volts.
Per Waveshare JetBot schematic, the ADC is reading 1/4 of +12VCC bus

Voltage output is (reading/32748)*4.096*4
'''
FACTOR=4*4.096/32748

class Battery():
    def __init__(self):
        class Devicex(Device):
            def readList(self, register, length):
                if sys.version_info < (3, 0):
                    register = chr(register)

                return super(Devicex, self).readList(register, length)

        def get_i2c_device(address, busnum=1, i2c_interface=None, **kwargs):
            """Return an I2C device for the specified address and on the specified bus.
            If busnum isn't specified, the default I2C bus for the platform will attempt
            to be detected.
            """
            return Devicex(address, busnum, i2c_interface, **kwargs)

        class ADS1115x(ADS1115):
            def __init__(self, address=0x48, **kwargs):
                self._device = get_i2c_device(address, **kwargs)

        self._adc = ADS1115x()

    def __call__(self):
        return round(FACTOR * self._adc.read_adc(0, gain=1), 2)

if __name__ == '__main__':
    battery = Battery()
    print(battery())
