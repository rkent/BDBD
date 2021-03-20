# demo of getting depth info from ros depth image message

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from bdbd_common.utils import fstr, sstr

# see test/realsense to get this value using pyrealsense2
DEPTH_SCALE = 0.000125
DEPTH_CAMERA = '/sr305/depth/image_rect_raw'

cvBridge = CvBridge()

def on_depth_image(image_msg):
    image_np = cvBridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')
    shape = image_np.shape
    depth = image_np[shape[0]/2, shape[1]/2]
    print('shape', shape, 'depth (mm)', depth)

def main():
    rospy.init_node('rosdepth')
    cam_sub = rospy.Subscriber(DEPTH_CAMERA, Image, on_depth_image)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    main()