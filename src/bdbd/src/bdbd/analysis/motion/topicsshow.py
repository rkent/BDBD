'''
Reads and shows motion data topics
'''
import os
import rospy
import time
from bdbd.msg import MotorsRaw
from nav_msgs.msg import Odometry
import numpy as np

left = 0.0
right = 0.0
last_left = left
last_right = right
last_time = 0.0
first_time = None
showme = 0.40
combine = 2 # number of points to average

inputs = [] # left, right motor
outputs = [] # pose.position.x, .y, .z, .orientation.x, .y, .z, .w, twist.linear.x, .y, .z, .angular.x, .y, .z

output_sums = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
input_sums = [0, 0]
time_sums = 0.0
combine_count = 0

def odom_cb(msg):
    global last_left
    global last_right
    global last_time
    global first_time
    global output_sums, input_sums, time_sums, combine_count
    global inputs, outputs

    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    l = msg.twist.twist.linear
    a = msg.twist.twist.angular
    output_sums[0] += p.x
    output_sums[1] += p.y
    output_sums[2] += p.z
    output_sums[3] += o.x
    output_sums[4] += o.y
    output_sums[5] += o.z
    output_sums[6] += o.w
    output_sums[7] += l.x
    output_sums[8] += l.y
    output_sums[9] += l.z
    output_sums[10] += a.x
    output_sums[11] += a.y
    output_sums[12] += a.z
    print('l:{:6.3f} r:{:6.3f} vx:{:6.3f} vy:{:6.3f} px:{:6.3f} py:{:6.3f}'.format(left, right, l.x, l.y, p.x, p.y))
        
    #rospy.loginfo('bag_time: {}'.format(bag_time))

def motors_cb(msg):
    global left
    global right
    left = msg.left
    right = msg.right

rospy.init_node('motionbag')
rospy.Subscriber('/t265/odom/sample', Odometry, odom_cb)
rospy.Subscriber('/bdbd/motors/cmd_raw', MotorsRaw, motors_cb)
rospy.loginfo('{} starting with PID {}'.format(os.path.basename(__file__), os.getpid()))
try:
    rospy.spin()
except:
    pass
