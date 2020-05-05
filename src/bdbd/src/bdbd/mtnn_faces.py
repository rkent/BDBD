# use mtnn from tensorrt_demos to detect faces in an image
import os
import traceback
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from bdbd.libpy.MessageClient import MessageClient, packId, unpackId
import cv2
import numpy as np
import pickle
from cv_bridge import CvBridge
cv_bridge = CvBridge()
try:
    from Queue import Queue
except:
    from queue import Queue

BBOX_COLOR = (0, 255, 0)  # green
def show_faces(img, boxes):
    """Draw bounding boxes on image."""
    for bb in boxes:
        x1, y1, x2, y2, confidence = int(bb[0]), int(bb[1]), int(bb[2]), int(bb[3]), bb[4]
        cv2.rectangle(img, (x1, y1), (x2, y2), BBOX_COLOR, 2)
    return img

def extract_faces(image, boxes, min_size=40, min_confidence=.50):
    required_size = (160, 160)
    faces = []
    results = []
    for bb in boxes:
        try:
            x1, y1, x2, y2, confidence = int(bb[0]), int(bb[1]), int(bb[2]), int(bb[3]), bb[4]
            width = abs(x2 - x1)
            height = abs(y2 - y1)
            x1 = min(x1, x2)
            y1 = min(y1, y2)
            if width < min_size or height < min_size or confidence < min_confidence:
                continue

            # extend the box size to give more complete portrait
            xL = int(abs(x1) - .15 * width)
            xR = int(abs(x1) + 1.15 * width)
            yB = int(abs(y1) - .20 * height)
            yT = int(abs(y1) + 1.05 * height)
            # Resize smaller dimension (usually X) to have a square image
            height1 = yT - yB
            width1 = xR - xL
            hw = max(height1, width1)
            hw2 = int(hw / 2)
            xc = int((xR + xL) / 2)
            yc = int((yT + yB) / 2)
            # avoid edges
            xc = min(max(hw2, xc), image.shape[1] - hw2 - 2)
            yc = min(max(hw2, yc), image.shape[0] - hw2 - 2)

            result = (xc - hw2, yc - hw2, xc + hw2, yc + hw2, confidence)
            # extract the face
            face = image[(yc-hw2):(yc+hw2), (xc-hw2):(xc+hw2)]
            # resize pixels to the model size
            face = cv2.resize(face, required_size)
        except:
            exc = traceback.format_exc()
            log.warning('Exception caught, recovery is null result:\n%s', exc)
            face = None
            result = None
        faces.append(face)
        results.append(result)
    return faces, results

class FaceDetect():
    def __init__(self):
        self.facedetect_request_sub = rospy.Subscriber('facedetect/request', Empty, self.on_facedetect_request)
        self.image_in_sub = None # used for one-off subscriptions to grab an image
        self.facedetect_image_out_pub = rospy.Publisher('/facedetect/image_out', Image, queue_size=10)
        self.queue = Queue()
        self.mc = MessageClient('localhost', 'beedee-mtnn_faces', self.queue)
        self.mc.connect()
        self.mc.subscribe('beedee/faces', msgType='json')
        self.waiting = False # are we waiting for a face detection?
        self.cv_image = None

    def on_facedetect_request(self, msg):
        rospy.loginfo('got facedetect request')
        self.image_in_sub = rospy.Subscriber('/jetbot_camera_1/raw', Image, self.on_image_in)

    def on_image_in(self, msg):
        if not self.waiting:
            #if self.image_in_sub is not None:
            #    self.image_in_sub.unregister()
            #    self.image_in_sub = None
            self.cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            jpeg = cv2.imencode('.jpg', self.cv_image)[1].tobytes()
            #pickled = pickle.dumps(self.cv_image)
            self.mc.publish('bddata/get_faces', jpeg, msgType='bytes')
            self.waiting = True
            rospy.logdebug('sent get_faces request')

    def run(self):
        while not rospy.is_shutdown():
            while not self.queue.empty():
                topic, message = self.queue.get()
                if topic == 'beedee/faces':
                    self.waiting = False
                    boxes = message
                    rospy.loginfo('found {} faces'.format(len(boxes)))
                    if self.cv_image is not None:
                        faces, exboxes  = extract_faces(self.cv_image, boxes)
                        show_faces(self.cv_image, exboxes)
                        ros_image = cv_bridge.cv2_to_imgmsg(self.cv_image, 'rgb8')
                        self.facedetect_image_out_pub.publish(ros_image)
                        self.cv_image = None
                rospy.sleep(.001)

def main():
    rospy.init_node('facedetects')
    fn = FaceDetect()
    fn.run()

if __name__ == '__main__':
    main()