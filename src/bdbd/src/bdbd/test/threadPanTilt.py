# testing of a thread to manage pan/tilt

import rospy
from bdbd.libpantilt.PCA9685 import PCA9685
import random
import traceback
import threading
from bdbd_common.msg import PanTilt
try:
    from Queue import Queue, Empty
except:
    from queue import Queue, Empty


pan = 90.
tilt = 45.
PANTILT_RATE = 100 # hz rate of updates
PANTILT_DP = 1 # maximum pixels per interval

### pan/tilt hat
panTilt = PCA9685()
panTilt.setPWMFreq(50)

panTiltQueue = Queue()
pantiltThread = None

# this has been slew limited on the theory that rapid slews are causing power glitches that is crashing the nano
def do_pantilt():
    global pan
    global tilt

    while True:
        panTiltMsg = panTiltQueue.get()
        rospy.loginfo('PanTilt msg {} {} current pan,tilt {} {}'.format(panTiltMsg.pan, panTiltMsg.tilt, pan, tilt))
        target_pan = panTiltMsg.pan
        target_tilt = panTiltMsg.tilt
        while pan != target_pan or tilt != target_tilt:
            if pan < target_pan:
                pan = min(target_pan, pan + PANTILT_DP)
            else:
                pan = max(target_pan, pan - PANTILT_DP)

            if tilt < target_tilt:
                tilt = min(target_tilt, tilt + PANTILT_DP)
            else:
                tilt = max(target_tilt, tilt - PANTILT_DP)

            try:
                panTilt.setRotationAngle(1, max(0.0, min(180.0, pan)))
                panTilt.setRotationAngle(0, max(0.0, min(90.0, tilt)))

            except:
                rospy.logerr(traceback.format_exc())

            ptRate.sleep()

MAX_DPAN = 170
MIN_DPAN = 10
MAX_DTILT = 80
MIN_DTILT = 10

rospy.init_node('threadPanTilt')
ptRate = rospy.Rate(PANTILT_RATE)
pantiltThread = threading.Thread(target=do_pantilt)
pantiltThread.start()
while not rospy.is_shutdown():
    rpan = random.randint(MIN_DPAN, MAX_DPAN)
    rtilt = random.randint(MIN_DTILT, MAX_DTILT)
    panTiltQueue.put(PanTilt(rpan, rtilt))
    rospy.sleep(1.0)
