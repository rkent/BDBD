# test of laserscan output
from sensor_msgs.msg import LaserScan
from bdbd_common.utils import fstr
import rospy

rospy.init_node('laserscan')

def on_laserscan(msg):
    print(fstr(msg.ranges))

rospy.Subscriber('/pc_to_ls/scan', LaserScan, on_laserscan)

rospy.spin()
