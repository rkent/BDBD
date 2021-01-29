# testing of sschangepose action

import numpy as np
#from bdbd_common.pathPlan2 import speedPlan
import rospy
from actionlib import SimpleGoalState, SimpleActionClient
import time
import sys
from bdbd.msg import SsMotionGoal, SsMotionAction, SsMotionFeedback
from geometry_msgs.msg import Pose, Twist
from bdbd_common.utils import fstr
from bdbd_common.geometry import pose2to3, pose3to2, D_TO_R, transform2d
from nav_msgs.msg import Odometry
from bdbd.msg import MotorsRaw
from actionlib_msgs.msg import GoalStatus

sname = SimpleGoalState.to_string

class SsMotionClient:
    def __init__(self):
        self.motor_pub = None
        #self.motor_pub = rospy.Publisher('/bdbd/motors/cmd_raw', MotorsRaw, queue_size=10)
        self.current_odom = None
        self.fcount = 0
        self.result = SsMotionFeedback()
        # dynamic model estimation
        self.q = 8.0
        self.lrs = []
        self.qvx = []
        self.qvy = []
        self.qom = []

    def feedback_cb(self,f):
        self.fcount += 1
        self.result = f
        #self.motor_pub.publish(left, right)
        if self.fcount % 20 == 0:
            #print(f)
            self.lrs.append([f.left, f.right])
            self.qvx.append(self.q * f.twist_r[0])
            self.qvy.append(self.q * f.twist_r[1])
            self.qom.append(self.q * f.twist_r[2])
            if len(self.lrs) > 10:
                # estimate bx
                (bx, residuals, rank, s) = np.linalg.lstsq(np.array(self.lrs), np.array(self.qvx))
                (by, residuals, rank, s) = np.linalg.lstsq(np.array(self.lrs), np.array(self.qvy))
                (bo, residuals, rank, s) = np.linalg.lstsq(np.array(self.lrs), np.array(self.qom))
                print(fstr({'bx': bx, 'by': by, 'bo': bo}))
        return

    def odom_cb(self, odometry):
        self.current_odom = odometry
        return

    def done_cb(self, status, result):
        print('done status: {}\nresult:\n{}'.format(status, result))
        simple_state = self.client.wait_for_result()
        print('simple_state is {}'.format(sname(simple_state)))
        self.stop()

    def stop(self):
        # stop activity
        try:
            if self.odom_sub:
                self.odom_sub.unregister()
        except:
            pass
        if self.motor_pub:
            self.motor_pub.publish(0.0, 0.0)
        rospy.sleep(.1)

    def __call__(self, dx, dy, dtheta, vcruise):
        print('ssClient')
        self.odom_sub = rospy.Subscriber('/t265/odom/sample', Odometry, self.odom_cb)
        # wait for messages
        time.sleep(.1)

        self.client = SimpleActionClient('/bdbd/ssmotion', SsMotionAction)
        print('waiting for action server')
        self.client.wait_for_server()
        sp = self.current_odom.pose.pose
        st = self.current_odom.twist.twist
        start_m = pose3to2(sp)
        # ending delta is given relative to current robot frame
        frame_m = (0.0, 0.0, 0.0)
        start_r = transform2d(start_m, frame_m, start_m)
        end_r = (start_r[0] + dx, start_r[1] + dy, start_r[2] + dtheta)
        end_m = transform2d(end_r, start_m, frame_m)

        ep = pose2to3(end_m)
        et = Twist()
        print('start pose: ' + fstr(pose3to2(sp)) + ' end pose: ' + fstr(pose3to2(ep)))
        goal = SsMotionGoal()
        goal.startPose = sp
        goal.endPose = ep
        goal.startTwist = st
        goal.endTwist = et
        goal.vcruise = vcruise
        goal.skip = 4
        goal.do_motor = True
        print(goal)
        self.client.send_goal(goal, feedback_cb=self.feedback_cb, done_cb=self.done_cb)
        print('goal started, waiting for result')

def shutdown_cb():
    print('shutdown_cb')
    if ssMotionClient.motor_pub:
        ssMotionClient.motor_pub.publish(0.0, 0.0)
    if ssMotionClient.client.simple_state == SimpleGoalState.ACTIVE:
        print('cancelling')
        ssMotionClient.client.cancel_goal()

if __name__ == '__main__':
    ssMotionClient = SsMotionClient()
    rospy.init_node('test_ssmotion')
    rospy.on_shutdown(shutdown_cb)
    ssMotionClient(.3, .0, 90.0 * D_TO_R, .30)
    while (not rospy.is_shutdown()):
        try:
            rospy.sleep(10.0)
            print('status: {}'.format(sname(ssMotionClient.client.simple_state)))
            rospy.signal_shutdown("Done")
        except:
            print('Other error')
            print(sys.exc_info())
            rospy.signal_shutdown("Error")
            break
    rospy.sleep(.1) 
    print('Final exit')