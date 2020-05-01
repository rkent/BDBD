#!/usr/bin/env python
import rospy

from bdbd.msg import PanTilt
from libpantilt.PCA9685 import PCA9685
panTilt = PCA9685()
panTilt.setPWMFreq(50)

class CenterPanTilt():
    pan = 90.0
    tilt = 45.0

def on_pantilt(msg):
    panTilt.setRotationAngle(1, max(0.0, min(180.0, msg.pan)))
    panTilt.setRotationAngle(0, max(0.0, min(90.0, msg.tilt)))

def main():
    rospy.init_node('pantilt')
    sub = rospy.Subscriber('pantilt', PanTilt, on_pantilt)

    # initialize pan and tilt
    on_pantilt(CenterPanTilt)

    # start running
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # center pan/tilt before exiting
        on_pantilt(CenterPanTilt)

if __name__ == '__main__':
    main()
