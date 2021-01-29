# maintains the robot dynamic model by storing motion vs left, right
import rospy
import os
from bdbd_common.geometry import LrDynamicsStore

def main():
    rospy.init_node('lrcapture')
    rospy.loginfo('{} starting with PID {}'.format(os.path.basename(__file__), os.getpid()))
    filedir = os.path.dirname(os.path.abspath(__file__)) + '/bddata'
    lrd = LrDynamicsStore(filedir)
    rospy.Timer(rospy.Duration(2), lrd.save)
    collect_rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        subscribed = True
        if not (lrd.motor_sub and lrd.odom_sub):
            subscribed = lrd.subscribe()
        if subscribed:
            lrd.collect()
        collect_rate.sleep()

if __name__ == '__main__':
    main()
