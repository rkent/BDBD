#!/usr/bin/env python
'''
    This ROS node processed classification information from ros_deep_learning.
'''
import rospy
import time
import traceback

from vision_msgs.msg import VisionInfo, Classification2D

class ClassInfo():
    msg = None
    @class_method
    def info_cb(cls, msg):

    @class_method
    def get_classes(cls):
        info_sub = rospy.Subscriber('/imagenet/vision_info', VisionInfo, cls.info_cb)
        
