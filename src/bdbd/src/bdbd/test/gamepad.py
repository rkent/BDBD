#!/usr/bin/env python
from bdbd.libpy.libgamepad import GamePad
import time

def publishEvents():
    pad = GamePad()
    while True:
        result = pad.getEvent()
        if result:
            print(str(result))
        time.sleep(.01)

def main():
    try:
        publishEvents()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()