#!/usr/bin/env python

# Test of relationship between system and ros time

import time
import rospy
from bdbd_common.utils import fstr
from nav_msgs.msg import Odometry

def odom_cb(odometry):
    global ros_start
    global sys_start
    now = float(odometry.header.stamp.secs + 1.0e-9 * odometry.header.stamp.nsecs)
    if ros_start is None:
        ros_start = now
        sys_start = time.time()
    else:
        lag = (time.time() - sys_start) - (now - ros_start)
        print(fstr({'lag ms': lag * 1000}))

# start executing the action, driven by odometry message receipt
ros_start = None
sys_start = None
rospy.init_node('timetest')
rate = rospy.Rate(100)
while (not rospy.is_shutdown()):
    sys = time.time()
    ros = rospy.get_time()
    print(fstr({'sys': sys, 'ros': ros, 'dif': sys-ros}))
    rate.sleep()
