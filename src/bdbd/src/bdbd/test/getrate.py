# receives a message at a fixed rate for timing tests
import rospy
from nav_msgs.msg import Odometry
from bdbd_common.utils import fstr
import time

def sub_cb(odom):
    then = float(odom.header.stamp.secs + 1.0e-9 * odom.header.stamp.nsecs)
    now = rospy.get_time()
    lag = now - then
    print(fstr({'lag ms': lag * 1000, 'seq': odom.header.seq}) + 'now: {:20.3f}'.format(now))

rospy.init_node('getrate')

sub = rospy.Subscriber('/t265/odom/sample', Odometry, sub_cb, queue_size=10, tcp_nodelay=True)
rospy.spin()
