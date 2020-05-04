# handles errors with rate limiting

import time
import logging
import sys
import traceback

class Errors():
    def __init__(self, log=None, name=None, eph=10):
        if name is None:
            self._name = sys._getframe().f_back.f_code.co_name
        else:
            self._name = name
        self._eph = eph
        if log is None:
            self._log = logging.getLogger(self._name)
        else:
            self._log = log
        self._clockStart = time.time()
        self._errorCount = 0
    
    def handle(self):
        exc = traceback.format_exc()
        doExit = False
        self._errorCount +=1
        if self._errorCount > self._eph:
            if time.time() > self._clockStart + 3600:
                doExit = False
            else:
                doExit = True
            self._errorCount = 0
            self._clockStart = time.time()
        self._log.error(traceback.format_exc())
        if doExit:
            self._log.critical('Errors per hour exceeded, recommending exit')
        return doExit

# test and demo
if __name__ == '__main__':
    logging.basicConfig(level='DEBUG')
    log = logging.getLogger('testHandleErrors')
    errors = Errors(log, eph=3)
    while True:
        try:
            time.sleep(1)
            raise Exception('This is an exception')
        except:
            if errors.handle():
                break
            else:
                log.info(f'{__name__} ignoring error')
