#!/usr/bin/env python
import rospy
from bdbd_gamepad.msg import GamepadEvent
from gamepad import GamePad

def publishEvents():
    rospy.init_node('bdbd_gamepad')
    pub = rospy.Publisher('~events', GamepadEvent, queue_size=10)
    pad = GamePad()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        result = pad.getEvent()
        if result:
            print('event: ' + str(result))
            pub.publish(result[0], result[1])
        else:
            rate.sleep()
    print('rospy shutdown')

if __name__ == '__main__':
    try:
        publishEvents()
    except rospy.ROSInterruptException:
        pass
    