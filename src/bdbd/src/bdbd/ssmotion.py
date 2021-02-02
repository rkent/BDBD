#!/usr/bin/env python

# ROS node to implement ssmotion action server using state space control

# generally follows http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29

import numpy as np

'''
https://github.com/python-control/Slycot/issues/15

What we do for testing in python-control is to use the following
set of commands to install slycot:

sudo apt-get install gfortran liblapack-dev

either:
git clone https://github.com/python-control/Slycot.git slycot
cd slycot
sudo python setup.py install

or:
pip install slycot

'''

import os
import rospy
import actionlib
import traceback
import math
from bdbd.msg import SsResults, SsMotionFeedback, SsMotionResult, SsMotionAction, MotorsRaw
from bdbd_common.pathPlan2 import static_plan
from bdbd_common.utils import fstr
from bdbd_common.ssControl import SsControl
from bdbd_common.geometry import pose3to2, twist3to2, LrDynamicsStore

from nav_msgs.msg import Odometry

ODOM_RATE = 1. / 200.

class ChangePose():
    def __init__(self):
        self._actionServer = actionlib.SimpleActionServer('ssmotion', SsMotionAction, auto_start=False)
        self._actionServer.register_goal_callback(self.goal_cb)
        self._actionServer.register_preempt_callback(self.preempt_cb)
        self._actionServer.start()
        self.ssResultsPub = rospy.Publisher('ssmotion/ssresults', SsResults, queue_size=10)
        self.count = 0
        self.skip = 1
        self.psum = [0.0, 0.0, 0.0, 0.0] # x, y, cos, sin
        self.tsum = [0.0, 0.0, 0.0]
        self.tt = 0.0
        self.ssControl = None
        self.motor_pub = None
        self.odom_sub = None
        self.results = None
        self.replan_count = 0
        self.replan_time = 0
        self.vslow = 0.10
        self.new_factor = 1.0
        self.last_left = 0.0
        self.last_right = 0.0

    def preempt_cb(self):
        rospy.loginfo('goal preempted')
        self._actionServer.set_preempted()
        #traceback.print_stack()
        if self.odom_sub:
            self.odom_sub.unregister()
            self.odom_sub = None
        if self.motor_pub:
            self.motor_pub.publish(0.0, 0.0)
            # unfortunately rospy does not properly clean up subscribers :(
            #self.motor_pub.unregister()
            self.motor_pub = None

        return

    def goal_cb(self):
        rospy.loginfo('accept new goal')
        goal = self._actionServer.accept_new_goal()
        self.goal = goal
        self.last_time = None
        self.skip = goal.skip
        self.dt = self.skip * ODOM_RATE
        if goal.do_motor:
            self.motor_pub = rospy.Publisher('/bdbd/motors/cmd_raw', MotorsRaw, queue_size=10)
        else:
            self.motor_pub = None

        filedir = os.path.dirname(os.path.abspath(__file__)) + '/bddata'
        lrd = LrDynamicsStore(filedir)
        lr_model = lrd.get_lr_model()
        self.ssControl = SsControl(lr_model=lr_model)

        # convert to 2D coordinates x, y, theta. The pose coordinates are in the frame which will
        # be considered the base frame for these calculations.
        start_m = pose3to2(goal.startPose)
        end_m = pose3to2(goal.endPose)
        self.pp = static_plan(start_pose=start_m
            , target_pose=end_m
            , start_twist=twist3to2(goal.startTwist)
            , target_twist=twist3to2(goal.endTwist)
            , approach_rho=0.20
            , min_rho=0.10
            , cruise_v=goal.vcruise
            , vslow=self.vslow
            , u=0.50
            , details=True
        )
        self.replan_time = rospy.get_time()

        rospy.loginfo('Executing changePose with goal x: {:6.3f} y: {:6.3f} theta: {:6.3f}'.format(*end_m))

        # start executing the action, driven by odometry message receipt
        self.odom_sub = rospy.Subscriber('/t265/odom/sample', Odometry, self.odom_cb, tcp_nodelay=True)

    def odom_cb(self, odometry):

        pose_m = pose3to2(odometry.pose.pose)
        twist_r = twist3to2(odometry.twist.twist)

        self.count += 1

        self.psum[0] += pose_m[0]
        self.psum[1] += pose_m[1]
        self.psum[2] += math.cos(pose_m[2])
        self.psum[3] += math.sin(pose_m[2])

        for i in range(3):
            self.tsum[i] += twist_r[i]

        if self.count % self.skip != 0:
            return

        then = float(odometry.header.stamp.secs + 1.0e-9 * odometry.header.stamp.nsecs)
        if self.last_time is None:
            self.last_time = then
            return

        pasum = []
        tasum = []
        for i in range(3):
            pasum.append(self.psum[i] / self.skip)
            tasum.append(self.tsum[i] / self.skip)
        pasum.append(self.psum[3] / self.skip)

        self.psum = [0.0, 0.0, 0.0, 0.0]
        self.tsum = [0.0, 0.0, 0.0]
        pose_m = (pasum[0], pasum[1], math.atan2(pasum[3], pasum[2]))

        dt = then - self.last_time
        self.tt += dt
        self.last_time = then
        lag = rospy.get_time() - then

        #rospy.loginfo('ssControl: ' + fstr({'lag': lag, 'dt': self.dt, 'tt': self.tt, 'pose_m': pose_m, 'twist_r': tasum}))

        (lr, ctl_state) = self.ssControl(self.dt, pose_m, tasum, self.pp)  

        eps = ctl_state['eps']
        pos_err = math.sqrt(0.33 * (eps[3]**2 + eps[4]**2 + eps[5]**2))
        ssResults = SsResults()

        # damping, used mostly when replanning
        self.new_factor = min(1.0, self.new_factor + .04)

        ssResults.left = self.new_factor * lr[0] + (1. - self.new_factor) * self.last_left
        ssResults.right = self.new_factor * lr[1] + (1. - self.new_factor) * self.last_right
        #print(fstr({'new_factor': self.new_factor, 'lr[0]': lr[0], 'last_left': self.last_left, 'left': ssResults.left}))
        self.last_left = ssResults.left
        self.last_right = ssResults.right
        ssResults.rms_err = ctl_state['rms_err']
        ssResults.eps = ctl_state['eps']
        ssResults.pose_m = pose_m
        ssResults.twist_r = twist_r
        ssResults.fraction = ctl_state['vv']['fraction']
        if self.motor_pub:
            self.motor_pub.publish(ssResults.left, ssResults.right)
        self.ssResultsPub.publish(ssResults)
        self._actionServer.publish_feedback(ssResults)
        self.results = ssResults

        #(pose_m, twist_m, twist_r) = dynamicStep(lr, dt, pose_m, twist_m)
        lrc = ctl_state['lr_corr']
        lrb = ctl_state['lr_base']
        #rospy.loginfo(fstr({'pos_err': pos_err, 'lag': lag, 'ctl_left': lr[0], 'ctl_right': lr[1]}))
        #rospy.loginfo(fstr({'L0': lrb[0], 'R0': lrb[1], 'corr_left': lrc[0], 'corr_right': lrc[1]}))

        if ssResults.fraction > 0.999 and self._actionServer.is_active():
            rospy.loginfo('fraction > 0.999, finishing goal')
            self.finish()

        # replanning
        time_since_replan = rospy.get_time() - self.replan_time
        if ssResults.fraction < 0.90 and pos_err > 0.10 and self.replan_count < 4 and time_since_replan > 2.0:
            self.replan_count += 1
            self.replan_time = rospy.get_time()
            self.new_factor = 0.0
            rospy.loginfo('pos_err exceeds limits, replanning path')
            self.pp = static_plan(start_pose=pose_m
                , target_pose=pose3to2(self.goal.endPose)
                , start_twist=twist_r
                , target_twist=twist3to2(self.goal.endTwist)
                , approach_rho=0.20
                , min_rho=0.10
                , cruise_v=self.goal.vcruise
                , vslow=self.vslow
                , u=0.50
                , details=True
            )

    def finish(self):
        if self.odom_sub:
            self.odom_sub.unregister()
            self.odom_sub = None
        if self.motor_pub:
            self.motor_pub.publish(0.0, 0.0)
            self.motor_pub = None
        rospy.loginfo('Done')
        if self._actionServer.is_active():
            self._actionServer.set_succeeded(self.results)
        self.results = None

def main():
    rospy.init_node('ssmotion')
    rospy.loginfo('{} starting with PID {}'.format(os.path.basename(__file__), os.getpid()))
    server = ChangePose()
    rospy.on_shutdown(server.finish)
    rospy.spin()

if __name__ == '__main__':
    main()
