# use mtnn from tensorrt_demos to detect faces in an image
import os
import traceback
from subprocess import Popen
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from bdbd.msg import NameProb
from bdbd.libpy.MessageClient import MessageClient, packId, unpackId
import cv2
import numpy as np
import pickle
from cv_bridge import CvBridge
try:
    from Queue import Queue
except:
    from queue import Queue

cv_bridge = CvBridge()

BBOX_COLOR = (0, 255, 0)  # green

def show_name(image, bb, name, prob):
    x1, y1, x2, y2, confidence = int(bb[0]), int(bb[1]), int(bb[2]), int(bb[3]), bb[4]
    if name.startswith('Some'):
        name = name.split()[1]
    font                   = cv2.FONT_HERSHEY_SIMPLEX
    fontScale              = 1
    fontColor              = (255,0,255)
    lineType               = 8
    thickness              = 2

    cv2.putText(image, name, 
        (x1, y1), 
        font, 
        fontScale,
        fontColor,
        thickness)
    cv2.putText(image, str(round(prob, 1)),
        (x1, y2), 
        font, 
        fontScale,
        fontColor,
        thickness)
    return

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
        self.image_in_sub = rospy.Subscriber('/jetbot_camera_1/raw', Image, self.on_image_in)
        self.facedetect_image_out_pub = rospy.Publisher('/facedetect/image_out', Image, queue_size=10)
        self.facename_image_out_pub = rospy.Publisher('/facename/image_out', Image, queue_size=10)
        self.facename_out_pub = rospy.Publisher('/facedetect/facename', NameProb, queue_size=10)
        self.queue = Queue()
        self.mc = MessageClient('localhost', 'beedee-mtnn_faces', self.queue)
        self.mc.connect()
        self.mc.subscribe('beedee/faces', msgType='json')
        self.mc.subscribe('beedee/command')
        self.mc.subscribe('beedee/facename', msgType = 'json') # result from do_tf face naming
        self.waiting = True # are we waiting for a face detection?
        self.cv_image = None
        self.facename_image = None
        self.p_gpu = None
        self.p_tf = None
        self.face_names = [] # face box dimensions with attached names
        self.face_list = [] # list of faces needing name detection

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
        file_dir = os.path.dirname(os.path.abspath(__file__))
        do_gpu_path = file_dir + '/do_gpu.py'
        do_tf_path = file_dir + '/do_tf.py'

        self.p_gpu = Popen(['/usr/bin/python3', do_gpu_path])
        self.p_tf = Popen(['/usr/bin/python3', do_tf_path])

        try:
            while not rospy.is_shutdown():
                while not self.queue.empty():
                    topic, message = self.queue.get()

                    if topic == 'beedee/command':
                        if message == 'GET_EMBEDDING.READY':
                            rospy.loginfo('mtnn_faces ready for input')
                            self.waiting = False

                    if topic == 'beedee/faces':
                        self.waiting = False
                        boxes = message
                        rospy.logdebug('found {} faces'.format(len(boxes)))
                        if self.cv_image is not None:
                            faces, exboxes  = extract_faces(self.cv_image, boxes)
                            show_faces(self.cv_image, exboxes)
                            ros_image = cv_bridge.cv2_to_imgmsg(self.cv_image, 'rgb8')
                            self.facedetect_image_out_pub.publish(ros_image)
                            # initiate name assignment
                            if len(self.face_list) == 0:
                                self.facename_image = self.cv_image
                                for i in range(0, len(faces)):
                                    jpeg = cv2.imencode('.jpg', faces[i])[1].tobytes()
                                    self.face_list.append({'face': jpeg, 'box': exboxes[i]})
                                if len(self.face_list) > 0:
                                    self.mc.publish('beedee/get_facename', self.face_list[0]['face'], msgType='bytes')
            
                            self.cv_image = None

                    if topic == 'beedee/facename':
                        facename = message
                        name = facename['name']
                        prob = facename['probability']
                        facebox = self.face_list.pop(0)
                        box = facebox['box']
                        rospy.loginfo('Face name: {} prob: {:6.3f}'.format(name, prob))
                        show_name(self.facename_image, box, name, prob)
                        self.facename_out_pub.publish(name, prob)
                        if len(self.face_list) > 0:
                            self.mc.publish('beedee/get_facename', self.face_list[0]['face'], msgType='bytes')
                        else:
                            ros_image = cv_bridge.cv2_to_imgmsg(self.facename_image, 'rgb8')
                            self.facename_image_out_pub.publish(ros_image)
                rospy.sleep(.01)
        finally:
            self.mc.publish('beedee/command', 'do_gpu_quit')
            self.mc.publish('beedee/command', 'do_tf_quit')
            self.p_tf.wait()
            self.p_gpu.wait()
            self.mc.close()

def main():
    rospy.init_node('facedetects')
    fn = FaceDetect()
    fn.run()

if __name__ == '__main__':
    main()