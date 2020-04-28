'''
Interface to the gamepad supplied with the Waveshare jetbot

Reports with the name 'ShanWan PC/PS3/Android'
'''
#!pip install evdev
import os
from evdev import InputDevice, categorize, ecodes

class InputDevice2(InputDevice):
    # python2-compatible, see https://github.com/gvalkov/python-evdev/pull/119/commits/511beeb781c2ca0e8aa59d457861ddc12e85e790

    def close(self):
        if self.fd > -1:
            try:
                super(InputDevice, self).close()
                os.close(self.fd)
            finally:
                self.fd = -1

class GamePad():
    def __init__(self):
        self._device = None
        self.find()

    def find(self):
        fileNames = os.listdir('/dev/input')

        # locate the first device that has a BTN_GAMEPAD key
        foundIt = False
        device = None
        for fileName in fileNames:
            try:
                if not fileName.startswith('event'): continue
                try:
                    device = InputDevice2('/dev/input/' + fileName)
                    cap = device.capabilities()
                    if not ecodes.EV_KEY in cap: continue
                    if not ecodes.BTN_GAMEPAD in cap[ecodes.EV_KEY]: continue
                except:
                    continue
                foundIt = True
                break
            except:
                continue
        if foundIt:
            self._device = device
            return device
        else:
            print('No gamepad device found')
        return None

    def _getName(self, code):
        if code in ecodes.BTN:
            name = ecodes.BTN[code]
        elif code in ecodes.ABS:
            name = ecodes.ABS[code]
        elif code in ecodes.KEY:
            name = ecodes.KEY[code]
        else:
            name = 'UNKNOWN'
        # name could be a list. just use first
        if not isinstance(name, str):
            name = name[0]
        return name

    def getEvent(self):
        if self._device is None:
            raise Exception('Not initialized or device not found')
        while True:
            event = self._device.read_one()
            if not event:
                return event
            if event.type in (ecodes.EV_KEY, ecodes.EV_ABS):
                # get the name
                name = self._getName(event.code)
                return (name, event.value)

    def getActive(self):
        return map(self._getName, self._device.active_keys())

    def getAbs(self, name):
        return self._device.absinfo(ecodes.ecodes[name])

# test and demo of usage
if __name__ == '__main__':
    import time
    pad = GamePad()
    #device = pad.find()
    #print(device)
    while True:
        while True:
            result = pad.getEvent()
            if not result:
                break
            print('event: ' + str(result))
            #print(*[x for x in pad.getActive()])
            if result[0].startswith('ABS'):
                print(pad.getAbs(result[0]))
        time.sleep(.2)
