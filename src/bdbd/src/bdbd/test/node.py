# skeleton node for use in testing
import rospy
import os
import tf
import time
import math
from geometry_msgs.msg import PoseStamped
from bdbd.libpy.geometry import poseDistance, poseTheta, D_TO_R

def main():
    rospy.init_node('test')
    tfl = tf.TransformListener()
    time.sleep(1)
    while not rospy.is_shutdown():
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.orientation.w = 1.0
        pose.pose.position.x = 0.2
        pose.pose.position.y = 0.2
        base_target = tfl.transformPose('base_link', pose)
        tp = base_target.pose.position
        distance = math.sqrt(tp.y**2 + tp.x**2)
        print('\ndistance: {:6.3f}'.format(distance))
        print('base_target: {}'.format(base_target.pose))

        current_pose = PoseStamped()
        current_pose.header.frame_id = 'base_link'
        current_pose.pose.orientation.w = 1.0
        current_pose = tfl.transformPose('map', current_pose)
        print('current_pose: {}'.format(current_pose.pose))

        time.sleep(1)
    
main()
