#!/usr/bin/python3
# use gpu to process image using https://github.com/jkjung-avt/tensorrt_demos

import logging
import traceback
import os
import sys
import time
import logging
import pickle
from bdbd.libpy.MessageClient import MessageClient, packId, unpackId

def do_gpu():
    from queue import Queue
    queue = Queue()

    try:
        log_level = 'INFO'
        log_format = '%(name)-20s%(process)-8s%(levelname)-9s%(asctime)s: %(message)s'

        logging.basicConfig(level=log_level, format=log_format)
        log = logging.getLogger(__name__)

        mc = MessageClient('localhost', 'beedee-do_gpu', queue)
        mc.connect()
        mc.subscribe('beedee/command')
        mc.subscribe('bddata/get_faces', msgType='bytes')

        log.info(f'{__name__} starting with PID {str(os.getpid())}')

        # locate and load tensorrt_demos face detection
        start = time.time()
        log.info('starting import of mtcnn')
        file_dir = os.path.dirname(os.path.abspath(__file__))
        trt_dir = os.path.abspath(file_dir + '../../../../../' + 'tensorrt_demos')
        cwd = os.getcwd()
        os.chdir(trt_dir)
        sys.path.insert(0, trt_dir)
        from utils.mtcnn import TrtMtcnn
        mtcnn = TrtMtcnn()
        os.chdir(cwd)
        log.info(f'done with mtcnn import, elapsed time {round(time.time() - start, 3)}')

        log.info('do_gpu entering main loop')        
        mc.publish('beedee/command', 'GET_FACES.READY')
        while True:
            try:
                topic, message = queue.get()
                log.info('got topic {}'.format(topic))
                if topic == 'beedee/command':
                    if message == '_quit':
                        break

                if topic == 'bddata/get_faces':
                    log.info('get_faces request')
                    frame = pickle.loads(message, encoding='bytes')
                    boxes, landmarks = mtcnn.detect(frame, minsize=40)
                    mc.publish('beedee/faces', boxes.tolist(), msgType='json')
                    log.info('get_faces results {}'.format(boxes))
            except KeyboardInterrupt:
                break

    except:
        exc = traceback.format_exc()
        log.error(f'{__name__} error')
        log.error('Exception caught:\n%s', exc)
    finally:
        mc.close()
        log.info(f'{__name__} ending')
    return 0

if __name__ == '__main__':
    do_gpu()