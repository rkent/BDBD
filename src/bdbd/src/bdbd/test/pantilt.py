#!/usr/bin/env python
'''
    pantilt testing
'''
import rospy
import traceback

from bdbd_common.msg import PanTilt
from bdbd.libpantilt.PCA9685 import PCA9685

pan = 90.0
tilt = 45.0

D_TO_R = 3.1415926535 / 180. # degrees to radians

def main():

    ### pan/tilt functions
    class CenterPanTilt():
        ''' a PanTilt message preset to center pan/tilt '''
        pan = 90.0
        tilt = 45.0

    def on_pantilt(msg):
        rospy.loginfo('PanTilt msg {} {}'.format(msg.pan, msg.tilt))
        try:
            panTilt.setRotationAngle(1, max(0.0, min(180.0, msg.pan)))
            panTilt.setRotationAngle(0, max(0.0, min(90.0, msg.tilt)))

        except:
            rospy.logerr(traceback.format_exc())

    # setup ros node
    rospy.init_node('test_pantilt')

    ### pan/tilt hat
    try:
        panTilt = PCA9685()
        panTilt.setPWMFreq(50)
        # initialize pan and tilt
        on_pantilt(CenterPanTilt)
    except:
        rospy.logerr(traceback.format_exc())

    # start running
    try:
        while not rospy.is_shutdown():
            # vary tilt
            ptm = PanTilt()
            ptm.pan, ptm.tilt = input('pan, tilt:')
            on_pantilt(ptm)
            rospy.sleep(.1)

    except rospy.ROSInterruptException:
        pass
    finally:
        # center pan/tilt
        on_pantilt(CenterPanTilt)

if __name__ == '__main__':
    main()
