'''
Reads and saves data from a ROS bag (running separately) recording motion data as raw motor plus pose
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
combine = 5 # number of points to average

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

    bag_time = msg.header.stamp.secs + msg.header.stamp.nsecs / 1000000000.0
    if not first_time:
        first_time = bag_time
        rospy.loginfo('{}'.format(msg))
    '''
    bag_time = bag_time - first_time
    if last_left != left or last_right != right:
        last_left = left
        last_right = right
        rospy.loginfo('left: {} right: {}'.format(left, right))
        last_time = bag_time
    if bag_time < last_time + showme:
        rospy.loginfo('time: {:6.3f} dx: {:7.4f} z: {:7.4f}'.format(bag_time, msg.twist.twist.linear.x, msg.twist.twist.angular.z))
    '''
    combine_count += 1
    input_sums[0] += left
    input_sums[1] += right

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
    if combine_count == combine:
        combine_count = 0
        for i in range(0, len(output_sums)):
            output_sums[i] /= combine
        outputs.append(output_sums)
        rospy.loginfo('outputs length: {}'.format(len(outputs)))
        output_sums = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        for i in range(0, len(input_sums)):
            input_sums[i] /= combine
        inputs.append(input_sums)
        input_sums = [0, 0]
        
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

# convert to numpy and save
np_outs = np.array(outputs, np.float32)
np_ins = np.array(inputs, np.float32)

rospy.loginfo('np_outs shape: {}'.format(np_outs.shape))

np.save('data/outputs_fortieth.npy', np_outs)
np.save('data/inputs_fortieth.npy', np_ins)
