#!/usr/bin/env python

# ROS node to implement changePose action server using state space control

# generally follows http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Server%20using%20the%20Execute%20Callback%20%28Python%29

import numpy as np

'''
https://github.com/python-control/Slycot/issues/15

What we do for testing in python-control is to use the following
set of commands to install slycot:

sudo apt-get install gfortran liblapack-dev
git clone https://github.com/python-control/Slycot.git slycot
cd slycot
sudo python setup.py install
'''

import os
import time
import rospy
import actionlib
import traceback
import math
from bdbd.msg import ChangePoseFeedback, ChangePoseResult, ChangePoseAction
from bdbd_common.pathPlan2 import static_plan
from bdbd_common.utils import fstr
from bdbd_common.ssControl import SsControl
from bdbd_common.geometry import lr_est, default_lr_model, D_TO_R, transform2d, DynamicStep, Motor, pose3to2

from nav_msgs.msg import Odometry

class ChangePose():
    def __init__(self):
        self._feedback = ChangePoseFeedback()
        self._result = ChangePoseResult()
        self._actionServer = actionlib.SimpleActionServer('sschangepose', ChangePoseAction, auto_start=False)
        self._actionServer.register_goal_callback(self.goal_cb)
        self._actionServer.register_preempt_callback(self.preempt_cb)
        self._actionServer.start()
        self._motor = Motor()
        self.count = 0
        self.skip = 4
        self.psum = [0.0, 0.0, 0.0]
        self.tsum = [0.0, 0.0, 0.0]
        self.tt = 0.0
        self.ssControl = SsControl()

    def preempt_cb(self):
        print('preempted')
        traceback.print_stack()
        if hasattr(self, 'odom_sub'):
            self.odom_sub.unregister()
        return

    def goal_cb(self):
        print('accept new goal')
        goal = self._actionServer.accept_new_goal()
        self.last_time = None
        self.dt = goal.dt
        # convert to 2D coordinates x, y, theta. The pose coordinates are in the frame which will
        # be considered the base frame for these calculations.
        start_m = pose3to2(goal.startPose)
        end_m = pose3to2(goal.endPose)

        vhat_cruise = goal.vcruise
        '''
        vx_start = goal.startTwist.linear.x
        omega_start = goal.startTwist.angular.z
        vx_end = goal.endTwist.linear.x
        omega_end = goal.endTwist.angular.z
        '''
        zero3 = (0.0, 0.0, 0.0)
        self.pp = static_plan(start_pose=start_m
            , target_pose=end_m
            , start_twist=zero3
            , target_twist=zero3
            , approach_rho=0.20
            , min_rho=0.10
            , cruise_v=vhat_cruise
            , u=0.50
            , details=True
        )

        rospy.loginfo('Executing changePose with goal x: {:6.3f} y: {:6.3f} theta: {:6.3f}'.format(*end_m))

        # start executing the action, driven by odometry message receipt
        self.odom_sub = rospy.Subscriber('/t265/odom/sample', Odometry, self.odom_cb, tcp_nodelay=True)

    def odom_cb(self, odometry):
        then = float(odometry.header.stamp.secs + 1.0e-9 * odometry.header.stamp.nsecs)

        pose_m = pose3to2(odometry.pose.pose)
        twist3 = odometry.twist.twist
        twist_r = (twist3.linear.x, twist3.linear.y, twist3.angular.z)

        if self.last_time is None:
            self.last_time = then
            return

        self.count += 1

        for i in range(3):
            self.psum[i] += pose_m[i]
            self.tsum[i] += twist_r[i]

        if self.count % self.skip != 0:
            return

        pasum = []
        tasum = []
        for i in range(3):
            pasum.append(self.psum[i] / self.skip)
            tasum.append(self.tsum[i] / self.skip)

        self.psum = [0.0, 0.0, 0.0]
        self.tsum = [0.0, 0.0, 0.0]
        dt = then - self.last_time
        self.tt += dt
        self.last_time = then
        lag = rospy.get_time() - then

        (lr, ctl_state) = self.ssControl(self.dt, pasum, tasum, self.pp)  

        self._feedback.fraction = self.pp.lp_frac
        self._feedback.dy = self.pp.dy_r
        self._feedback.vnew = ctl_state['vv']['v']
        self._feedback.onew = ctl_state['vv']['omega']
        self._feedback.psi = self.pp.psi
        self._feedback.rostime = then
        self._feedback.tt = self.tt
        self._feedback.dhat = 0.0 # not implemented

        self._actionServer.publish_feedback(self._feedback)

        #(pose_m, twist_m, twist_r) = dynamicStep(lr, dt, pose_m, twist_m)
        lrc = ctl_state['lr_corr']
        lrb = ctl_state['lr_base']
        print(fstr({'lag': lag, 'L0': lrb[0], 'R0': lrb[1], 'corr_left': lrc[0], 'corr_right': lrc[1], 'ctl_left': lr[0], 'ctl_right': lr[1]}))

        if self.pp.lp_frac > 0.999 and self._actionServer.is_active():
            print(fstr({'lp_frac': self.pp.lp_frac}))
            self.finish()

    def finish(self):
        print('finish called')
        if hasattr(self, 'odom_sub'):
            self.odom_sub.unregister()

        self._result.yerror = .01
        self._result.terror = .02
        rospy.loginfo('Done')
        self._actionServer.set_succeeded(self._result)

def main():
    rospy.init_node('changepose')
    rospy.loginfo('{} starting with PID {}'.format(os.path.basename(__file__), os.getpid()))
    server = ChangePose()
    rospy.spin()

if __name__ == '__main__':
    main()
