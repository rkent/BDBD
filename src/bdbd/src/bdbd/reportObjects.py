#!/usr/bin/env python
'''
reads detectNet results and reports
'''

import rospy
import time
import traceback
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, VisionInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import cv2

SAY_RATE = 5.0 # time in seconds to repeat a seen category
MIN_SCORE = 0.90

bridge = CvBridge()
font = cv2.FONT_HERSHEY_PLAIN

class ReportResults():
    def __init__(self):
        rospy.init_node('reportObjects')
        rospy.loginfo('{} starting with PID {}'.format(os.path.basename(__file__), os.getpid()))
        # get the categories
        infoMsg = rospy.wait_for_message('/detectnet/vision_info', VisionInfo, timeout=None)
        self.categories = rospy.get_param(infoMsg.database_location)
        rospy.loginfo('categories: {}'.format(self.categories))
        self.detections_sub = rospy.Subscriber('/detectnet/detections', Detection2DArray, self.detections_cb)
        self.sayit_pub = rospy.Publisher('sayit/text', String, queue_size=10)
        self.image_sub = rospy.Subscriber('~image_in', Image, self.image_cb)
        self.image_pub = rospy.Publisher('~image_out', Image, queue_size=1)
        self.categoryHistory = {}
        rospy.spin()
        self.last_detections = None
        self.last_image = None

    def image_cb(self, msg):
        try:
            self.last_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr('Cv Bridge error: {}'.format(e))

    def detections_cb(self, msg):
        for detection in msg.detections:
            best_category = ''
            best_score = 0.0
            b = detection.bbox
            # rospy.loginfo('bbox: {}'.format(b))
            corner1 = (int(b.center.x - b.size_x/2), int(b.center.y - b.size_y/2))
            corner2 = (int(b.center.x + b.size_x/2), int(b.center.y + b.size_y/2))
            color = (255, 0, 255)
            thickness = 2
            # rospy.loginfo('overlay: {}, {}, {}, {}'.format(corner1, corner2, color, thickness))
            if self.last_image is not None:
                cv2.rectangle(self.last_image, corner1, corner2, color, thickness)

            for result in detection.results:
                id = result.id
                score = result.score
                if score > best_score:
                    best_score = score
                    best_category = self.categories[id]

                if score > MIN_SCORE:
                    if id in self.categoryHistory:
                        categoryRecord = self.categoryHistory[id]
                    else:
                        categoryRecord = {
                            'category': self.categories[id],
                            'firstSeen': time.time(),
                            'lastSeen': 0.0,
                            'lastSaid': 0.0
                        }
                        self.categoryHistory[id] = categoryRecord

                    categoryRecord['lastSeen'] = time.time()
                    if categoryRecord['lastSaid'] + SAY_RATE < time.time():
                        self.sayit_pub.publish('I see a ' + categoryRecord['category'])
                        categoryRecord['lastSaid'] = time.time() 
                        rospy.loginfo('Class: {} Score: {:6.3f}'.format(self.categories[id], score))

            if self.last_image is not None:
                # add category description to box
                cv2.putText(
                    self.last_image,
                    '{} {:5.3f}'.format(best_category, best_score),
                    (int(b.center.x - b.size_x/2 + 3), int(b.center.y - b.size_y/2) + 16),
                    font, 1, (0, 255, 0)
                )
        if self.last_image is not None:
            try:
                self.image_pub.publish(bridge.cv2_to_imgmsg(self.last_image, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr('Cv Bridge error: {}'.format(e))

def main():
    ReportResults()

if __name__ == '__main__':
    main()