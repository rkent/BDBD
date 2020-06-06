# adapted from jetson camera module by RKJ. Display camera using cv2.
from __future__ import print_function
import atexit
import cv2
import threading
import numpy as np
import time
import struct

def bgr8_to_jpeg(value, quality=75):
    # recover the timestamp from the raw image, and reinsert as jpeg metadata type \xe3
    b = bytearray(8)
    for i in range(8):
        b[i] = value[0, i, 0]
    jpeg = bytes(cv2.imencode('.jpg', value)[1])
    # add text as a test
    ss =  b'\xff\xd8\xff\xe3\x00\x0a' + b[0:8] + jpeg[2:]
    return ss

# todo: don't duplicate code
class Camera():
    # config
    width = 640
    height = 360
    fps = 21
    capture_width = 1280
    capture_height = 720
    number = 0
    flip = 0

    def __init__(self):
        self.image = np.empty((self.height, self.width, 3), dtype=np.uint8)
        atexit.register(self.stop)
        # Show images
        cv2.namedWindow('Camera', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Camera', self.image)

    def _capture_frames(self):
        while True:
            re, image = self.cap.read()
            if re:
                self.image = image
                cv2.imshow('Camera', self.image)
            else:
                print('Failed to get frame')
                break
                
    def _gst_str(self):
        return 'nvarguscamerasrc sensor-id=%d ! video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 ! nvvidconv flip-method=%d ! video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! videoconvert ! appsink' % (
                self.number, self.capture_width, self.capture_height, self.fps, self.flip, self.width, self.height)
    
    def start(self):
        if not hasattr(self, 'cap'): # moved from __init__ in original
            self.cap = cv2.VideoCapture(self._gst_str(), cv2.CAP_GSTREAMER)
            re, image = self.cap.read()
            if not re:
                raise RuntimeError('Could not read image from camera.')
            self.image = image

        if not self.cap.isOpened():
            self.cap.open(self._gst_str(), cv2.CAP_GSTREAMER)
        if not hasattr(self, 'thread') or not self.thread.isAlive():
            self.thread = threading.Thread(target=self._capture_frames)
            self.thread.start()

    def stop(self):
        print('stopping')
        if hasattr(self, 'cap'):
            self.cap.release()
        if hasattr(self, 'thread'):
            self.thread.join()
            
    def restart(self):
        self.stop()
        self.start()

# main

camera = Camera()
camera.start()

try:
    while True:
        time.sleep(.01)
        cv2.waitKey(25)
except:
    camera.stop()
    raise