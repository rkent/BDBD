#!/usr/bin/env python
import rospy
from bdbd.msg import GamepadEvent
from libgamepad import GamePad

def publishEvents():
    rospy.init_node('gamepad')
    pub = rospy.Publisher('~events', GamepadEvent, queue_size=10)
    pad = GamePad()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        result = pad.getEvent()
        if result:
            # rospy.loginfo(str(result))
            pub.publish(result[0], result[1])
        else:
            rate.sleep()
    print('rospy shutdown')

def main():
    # dummy
    import os
    print(os.getcwd())
    try:
        publishEvents()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()