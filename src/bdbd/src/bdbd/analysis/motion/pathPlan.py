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
        min_rho=0.05, # the smallest radius we allow in a path plan,
        rhohat=0.184  # tradeoff between speed and omega, see RKJ 2020-09-23 pp 33
    ):
        self.s_plan = None
        self.path = None,
        self.lr_model = lr_model
        self.approach_rho = approach_rho
        self.min_rho = min_rho
        self.rhohat = rhohat

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
            | x_r | = | cos(theta)   sin(theta) | X | x_b | 
            | y_r |   | -sin(theta)  cos(theta) |   | y_b |
        '''

        x_b = self.end_pose2d[0] - self.start_pose2d[0]
        y_b = self.end_pose2d[1] - self.start_pose2d[1]
        c_theta = math.cos(self.start_pose2d[2])
        s_theta = math.sin(self.start_pose2d[2])
        x_r = c_theta * x_b + s_theta * y_b
        y_r = -s_theta * x_b + c_theta * y_b

        print(fstr({'x_r': x_r, 'y_r': y_r, 'phi': phi / D_TO_R}))
        print()
        # do we have a valid 2-arc solution?
        path2a = nearPath(x_r, y_r, phi)
        print('2 segment solution')
        if path2a:
            for s in path2a:
                print(fstr(s))

        path3a = shortestPath(x_r, y_r, phi, self.approach_rho)
        print('3 segment solution')
        for s in path3a:
            print(fstr(s))

        if path2a and (
             path2a[0]['radius'] > self.min_rho and path2a[0]['radius'] < self.approach_rho
        ):
            # use the nearPath solution
            path3a = None
        else:
            # get the 3 arc solution
            path2a = None

        self.path = path2a or path3a
        return self.path

    def speedPlan(self, vhat0, vhatcruise, vhatn, u=0.25):
        '''
        Given the path plan, determine a speed plan.

        vhat0: starting speed
        vhatcruise: cruising speed
        vhatn: final speed
        u: time for speed transition

        
        '''

        if self.path is None:
            raise Exception("You need to call start to determine the path plan before calling speedPlan")

        # calculate the total path length in transformed coordinates
        lprime_sum = 0.0
        for seg in self.path:
            if 'radius' in seg:
                lprime_sum += abs(seg['angle']) * (seg['radius'] + self.rhohat)
            else:
                lprime_sum += math.sqrt(
                    (seg['start'][0] - seg['end'][0])**2 +
                    (seg['start'][1] - seg['end'][1])**2
                )

        s_plan = None

        # See RKJ 2020-09-24 p36 for plan.
        # 1) Check for short path to target
        if vhat0 + vhatn != 0.0:
            dt = 2.0 * lprime_sum / (vhat0 + vhatn)
            if dt < 2.0 * u:
                s_plan = [
                    {
                        'start': 0.0,
                        'end': lprime_sum,
                        'vstart': vhat0,
                        'vend': vhatn,
                        'time': dt
                    }
                ]

        # 2) Ramp to a value, then ramp to vhatn
        if s_plan is None:
            vhatm = lprime_sum / u - 0.5 * (vhat0 + vhatn)
            if vhatm < vhatcruise:
                s_plan = [
                    {
                        'start': 0.0,
                        'end': lprime_sum / 2.0,
                        'vstart': vhat0,
                        'vend': vhatm,
                        'time': u
                    },
                    {
                        'start': lprime_sum / 2.0,
                        'end': lprime_sum,
                        'vstart': vhatm,
                        'vend': vhatn,
                        'time': u
                    }
                ]

        # 3) Add a length of vcruise in middle
        if s_plan is None:
            lprime_0 = 0.5 * u * (vhat0 + vhatcruise)
            lprime_n = 0.5 * u * (vhatn + vhatcruise)
            s_plan = [
                {
                    'start': 0.0,
                    'end': lprime_0,
                    'vstart': vhat0,
                    'vend': vhatcruise,
                    'time': u
                },
                {
                    'start': lprime_0,
                    'end': lprime_sum - lprime_n,
                    'vstart': vhatcruise,
                    'vend': vhatcruise,
                    'time': (lprime_sum - lprime_n - lprime_0) / vhatcruise
                },
                {
                    'start': lprime_sum - lprime_n,
                    'end': lprime_sum,
                    'vstart': vhatcruise,
                    'vend': vhatn,
                    'time': u
                }
            ]

        self.s_plan = s_plan
        return s_plan

    def v(self, dt):
        # return the expected position and velocities at time dt from plan start

        if self.s_plan is None or self.path is None:
            raise Exception('speed or path plan missing')

        # determine the speed plan segment
        tt = 0.0
        seg_time = None
        seg_index = None
        rho = None
        for i in range(len(self.s_plan)):
            seg = self.s_plan[i]
            if dt > tt + seg['time']:
                tt += seg['time']
                continue
            seg_time = dt - tt
            seg_index = i
            break

        # get speed, s from s_plan
        if seg_index is None:
            seg_index = len(self.s_plan) - 1
            seg_time = self.s_plan[seg_index]['time']
            dt = 0.0
            for seg in self.s_plan:
                dt += seg['time']

        seg = self.s_plan[seg_index]
        vhat = seg['vstart'] + seg_time * (seg['vend'] - seg['vstart']) / seg['time']
        sprime = seg['start'] + 0.5 * seg_time * (seg['vstart'] + vhat)

        # get parameters from path plan
        s_sum = 0.0
        sprime_sum = 0.0
        v = None
        for i in range(len(self.path)):
            seg = self.path[i]
            if 'radius' in seg:
                rho = seg['radius']
                lprime = seg['length'] * (rho + self.rhohat) / rho
            else:
                rho = None
                lprime = seg['length']

            if sprime_sum + lprime > sprime:
                # this is the correct segment
                if rho is None:
                    v = vhat
                    omega = 0.0
                else:
                    v = vhat * rho / (rho + self.rhohat)
                    omega = math.copysign(v / rho, seg['angle'])
                break
            else:
                sprime_sum += lprime
                s_sum += seg['length']

        if v is None:
            if 'radius' in self.path[-1]:
                rho = self.path[-1]['radius']
                v = vhat * rho / (rho + self.rhohat)
                omega = math.copysign(v / rho, self.path[-1]['angle'])
            else:
                v = vhat
                omega = 0.0
                rho = None
        return {'time': dt, 'sprime': sprime, 'vhat': vhat, 'v': v, 'omega': omega, 'rho': rho}

    def m(self, v, omega):
        # return motor left, right for a given speed, rotation
        # See RKJ 2020-09-14 pp 25
        '''
        pxl, pxr, fx = self.lr_model[0]
        pol, por, fo = self.lr_model[2]
        a = pxl / fx
        b = pxr / fx
        c = pol / fo
        d = por / fo
        denom_left = b * d - a * c
        denom_right = b * c - a * d
        left = (v * d - omega * b) / (b * d - a * c))
        right = (v * c - omega * a) / (b * c - a * d))
        return (left, right)
        '''
        pass

pp = PathPlan(approach_rho=0.25, min_rho=0.05, rhohat=0.184)
start_theta_degrees = 0.0
end_theta_degrees = 0.0
start_x = 0.0
start_y = 0.0
end_x = 0.2
end_y = 0.01

start_theta = start_theta_degrees * D_TO_R
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

path_plan = pp.start(pose1, pose2)
print('path plan:')
for p in path_plan:
    print(fstr(p))

s_plan = pp.speedPlan(0.20, 0.30, 0.0, u=0.25)
print('speed plan:')
for s in s_plan:
    print(fstr(s))

dt = 0.05
tt = 0.0

speeds = []
while tt < 2.0:
    ss = pp.v(tt)
    speeds.append(ss)
    if ss['time'] != tt:
        break # at end of plan
    tt += dt

for ss in speeds:
    print(fstr(ss))