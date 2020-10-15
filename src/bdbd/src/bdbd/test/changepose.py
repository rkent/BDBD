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
        self.motor_pub = rospy.Publisher('/bdbd/motors/cmd_raw', MotorsRaw, queue_size=10)
        self.current_odom = None
        self.motor = Motor()
        self.fcount = 0
        self.sys_start = None
        self.ros_start = None

    def feedback_cb(self,f):
        self.fcount += 1
        if self.sys_start is None:
            self.sys_start = time.time()
            self.ros_start = f.rostime
        (left, right) = self.motor(f.vnew, f.onew)
        lag = (time.time() - self.sys_start) - (f.rostime - self.ros_start)
        self.motor_pub.publish(left, right)
        pose_m = pose3to2(self.current_odom.pose.pose)
        #if self.fcount % 5 == 0:
        print('tt: {:6.3f} '.format(f.tt) + fstr({
            'vnew': f.vnew,
            'onew': f.onew,
            'left': left,
            'right': right,
            'psi': f.psi / D_TO_R,
            'fraction': f.fraction,
            'dy': f.dy,
            'pose_m': pose_m,
            'lagms': lag * 1000.
        }))
        return

    def odom_cb(self, odometry):
        self.current_odom = odometry
        return

    def done_cb(self, state, result):
        print('done state: () result: {}'.format(state, result))
        self.client.wait_for_result()
        print('after waiting')
        self.odom_sub.unregister()
        print('after unregister')
        return

    def __call__(self, dx, dy, dphid, vcruise):
        print('changePoseClient')
        self.odom_sub = rospy.Subscriber('/t265/odom/sample', Odometry, self.odom_cb)
        # wait for messages
        time.sleep(.1)

        self.client = actionlib.SimpleActionClient('/bdbd/changepose', ChangePoseAction)
        self.client.wait_for_server()
        sp = self.current_odom.pose.pose
        st = self.current_odom.twist.twist
        start_m = pose3to2(sp)
        end_m = (start_m[0] + dx, start_m[1] + dy, start_m[2] + dphid * D_TO_R)
        ep = pose2to3(end_m)
        et = Twist()
        goal = ChangePoseGoal(sp, ep, st, et, vcruise)
        self.client.send_goal(goal, feedback_cb=self.feedback_cb, done_cb=self.done_cb)
        print('waiting for result')

if __name__ == '__main__':
    changePoseClient = ChangePoseClient()
    rospy.init_node('test_changepose')
    changePoseClient(.2, .1, 0.0, .3)
    while (not rospy.is_shutdown()):
        try:
            time.sleep(4.0)
            if changePoseClient.client.get_state() == GoalStatus.ACTIVE:
                changePoseClient.client.cancel_goal()
            print('status: {}'.format(changePoseClient.client.get_state()))
            changePoseClient.motor_pub.publish(0.0, 0.0)
            #print('Result: {}'.format(fstr(result)))
            #print("program interrupted before completion")
        except:
            print('Other error')
            print(sys.exc_info())