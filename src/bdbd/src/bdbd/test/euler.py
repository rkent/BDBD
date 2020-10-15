# test of 3d to 2d conversion of t265 data
import rospy
import actionlib
import time
import sys
from bdbd.msg import ChangePoseGoal, ChangePoseAction
from geometry_msgs.msg import Pose, Twist
from bdbd_common.utils import fstr
from bdbd_common.geometry import pose2to3, pose3to2, D_TO_R, Motor
from nav_msgs.msg import Odometry
from bdbd.msg import MotorsRaw
from actionlib_msgs.msg import GoalStatus

class ChangePoseClient:
    def __init__(self):
        self.current_odom = None

    def odom_cb(self, odometry):
        self.current_odom = odometry
        pose_m = pose3to2(odometry.pose.pose)
        print(pose_m)
        return

    def __call__(self):
        print('test of pose3to2')
        self.odom_sub = rospy.Subscriber('/t265/odom/sample', Odometry, self.odom_cb)

if __name__ == '__main__':
    changePoseClient = ChangePoseClient()
    rospy.init_node('test_changepose')
    changePoseClient()
    while (not rospy.is_shutdown()):
        time.sleep(0.1)
