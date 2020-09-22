# development of a path planning class.

import time
import math
from geometry_msgs.msg import Pose, Twist
from bdbd_common.utils import fstr
from bdbd_common.geometry import shortestPath, nearPath, D_TO_R, lrEstimate, dynamic_motion, default_lr_model, q_to_array, array_to_q
import tf.transformations as transformations
import tf

class PathPlan():
    def __init__(
        self,
        lr_model=default_lr_model(),
        approach_rho=0.2, # radius of planned approach
        min_rho=0.05, # the smallest radius we allow in a path plan
    ):
        self.lr_model = lr_model
        self.approach_rho = approach_rho
        self.min_rho=min_rho

    def start(self, start_pose, end_pose, start_twist=Twist(), end_twist=Twist(), start_t=None, motor_max = 0.9):
        self.start_pose = start_pose
        self.end_pose = end_pose
        self.start_twist = start_twist
        self.end_twist = end_twist
        self.start_t = time.time() if start_t is None else start_t
        self.motor_max = motor_max
        
        # convert to 2D coordinates x, y, theta. The pose coordinates are in the frame which will
        # be considered the base frame for these calculations.
        a_start = tf.transformations.euler_from_quaternion(q_to_array(self.start_pose.orientation))[2]
        a_end = tf.transformations.euler_from_quaternion(q_to_array(self.end_pose.orientation))[2]
        self.start_pose2d = [start_pose.position.x, start_pose.position.y, a_start]
        self.end_pose2d = [end_pose.position.x, end_pose.position.y, a_end]
        print(fstr({'start': self.start_pose2d, 'end': self.end_pose2d}))

        # Initial Motion Plan

        phi = self.end_pose2d[2] - self.start_pose2d[2]

        '''
            Rotation matrix from base to robot frame:
            | x_r | = | cos(phi)   sin(phi) | X | x_b | 
            | y_r |   | -sin(phi)  cos(phi) |   | y_b |
        '''

        x_b = self.end_pose2d[0] - self.start_pose2d[0]
        y_b = self.end_pose2d[1] - self.start_pose2d[1]
        c_phi = math.cos(phi)
        s_phi = math.sin(phi)
        x_r = c_phi * x_b + s_phi * y_b
        y_r = -s_phi * x_b + c_phi * y_b

        # do we have a valid 2-arc solution?
        path2a = nearPath(x_r, y_r, phi)
        if path2a and path2a['rho'] > self.min_rho:
            # use the nearPath solution
            path3a = None
        else:
            # get the 3 arc solution
            path2a = None
            path3a = shortestPath(x_r, y_r, phi, self.approach_rho)

        return path2a or path3a


pp = PathPlan()
end_theta_degrees = 45.0
start_x = 0.0
start_y = 0.0
end_x = .5
end_y = .1

start_theta = 0.0
end_theta = end_theta_degrees * D_TO_R
pose1 = Pose()
pose1.position.x = start_x
pose1.position.y = start_y
pose2 = Pose()
pose2.position.x = end_x
pose2.position.y = end_y
pose1.orientation = array_to_q(tf.transformations.quaternion_from_euler(0.0, 0.0, start_theta))
pose2.orientation = array_to_q(tf.transformations.quaternion_from_euler(0.0, 0.0, end_theta))
print(pose1)
print(pose2)

pp.start(pose1, pose2)