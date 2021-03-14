# timing of a single message subscribe/unsubscribe cycle.

from bdbd_common.messageSingle import messageSingle
from sensor_msgs.msg import CameraInfo, CompressedImage
import rospy
import time

rospy.init_node('timeSub')
while not rospy.is_shutdown():
    start = time.time()
    msg = messageSingle('/t265/fisheye1/image_raw/compressed', CompressedImage)
    elapsed = time.time() - start
    print('elapsed: {:6.3f}', elapsed)
    rospy.sleep(1.0)
