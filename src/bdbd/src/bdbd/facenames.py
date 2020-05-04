# process images to get faces and names
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import cv2
from cv_bridge import CvBridge
br = CvBridge()
try:
    from Queue import Queue
except:
    from queue import Queue

import os
import traceback

BBOX_COLOR = (0, 255, 0)  # green
def show_faces(img, boxes):
    """Draw bounding boxes on image."""
    for bb in boxes:
        x1, y1, x2, y2, confidence = int(bb[0]), int(bb[1]), int(bb[2]), int(bb[3]), bb[4]
        cv2.rectangle(img, (x1, y1), (x2, y2), BBOX_COLOR, 2)
    return img

class FaceName():
    def __init__(self):
        self.facename_request_sub = rospy.Subscriber('facename/request', Empty, self.on_facename_request)
        self.imagein_sub = None # used for one-off subscriptions to grab an image
        self.detectnet_image_in_pub = rospy.Publisher('/detectnet/image_in', Image, queue_size=10)
        self.detectnet_detections_sub = rospy.Subscriber('detectnet/detections', Detection2DArray, self.on_detectnet_detections)
        self.facename_image_out_pub = rospy.Publisher('/facename/image_out', Image, queue_size=10)
        self.current_frame = None
        self.overlayed_frame = None

    def on_facename_request(self, msg):
        rospy.loginfo('got facename request')
        self.image_in_sub = rospy.Subscriber('/jetbot_camera_1/raw', Image, self.on_image_in)

    def on_image_in(self, msg):
        rospy.loginfo('Got camera frame')
        if self.current_frame is None:
            self.current_frame = msg
        if self.image_in_sub is not None:
            self.image_in_sub.unregister()
            self.image_in_sub = None
        self.detectnet_image_in_pub.publish(msg)
    
    def on_detectnet_detections(self, msg):
        boxes = []
        if len(msg.detections) > 0:
            rospy.loginfo('Found {} faces'.format(len(msg.detections)))
            for detection in msg.detections:
                rospy.loginfo('results: {} bbox: {}'.format(detection.results[0].score, detection.bbox))
                box = detection.bbox
                x1 = box.center.x - box.size_x / 2
                x2 = box.center.x + box.size_x / 2
                y1 = box.center.y - box.size_y / 2
                y2 = box.center.y + box.size_y / 2
                boxes.append([x1, y1, x2, y2, 0.0])
            cv_image = br.imgmsg_to_cv2(self.current_frame, 'bgr8') # Convert the message to a new image
            show_faces(cv_image, boxes)
            outmsg = br.cv2_to_imgmsg(cv_image, 'bgr8')
        else:
            rospy.loginfo('No faces found')
            outmsg = self.current_frame
        
        self.facename_image_out_pub.publish(outmsg)
        self.current_frame = None

    def run(self):
        rospy.spin()
        
def main():
    rospy.init_node('facenames')
    fn = FaceName()
    fn.run()

if __name__ == '__main__':
    main()