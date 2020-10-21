#!/usr/bin/env python

# ROS node to implement changePose action server

# generally follows http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29

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
rospy.init_node('lagtest')
odom_sub = rospy.Subscriber('/t265/odom/sample', Odometry, odom_cb)
while (not rospy.is_shutdown()):
    time.sleep(.1)
