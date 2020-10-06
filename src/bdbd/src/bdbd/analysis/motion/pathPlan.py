# development of a path planning class.

import time
import math
from math import cos, sin, sqrt
from geometry_msgs.msg import Pose, Twist
from bdbd_common.utils import fstr
from bdbd_common.geometry import threeSegmentPath, twoArcPath, D_TO_R, HALFPI, \
    Motor, default_lr_model, q_to_array, array_to_q, \
    nearestLinePoint, nearestArcPoint, transform2d
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf

class PathPlan():
    '''
    Manage movement from a start pose, twist to an end pose, twist

    Frames of reference:

    Inputs (including poses, twists, and lr_model) should be in a consistent, stationary frame
    of reference. Typically that would be the pose, twist measurement point in a 'world' or 'map'
    frame of reference. I'll call this the 'm' frame (for measurement). No suffix on variables
    typically means this frame of reference, or suffix m for clarity. 

    Frames of reference and their suffixes:
        m   'measure':  stationary frame that world measurements are valid
        p   'plan':     stationary frame used as origin of path plan. Wheel pose at start of plan
        r   'robot':    moving frame where lr_model is valid (ie, the measurement point)
        w   'wheels':   moving frame centered on wheel, ie center of rotation

    '''
    def __init__(
        self,
        lr_model=default_lr_model(),
        approach_rho=0.2, # radius of planned approach
        min_rho=0.05, # the smallest radius we allow in a path plan,
        rhohat=0.184  # tradeoff between speed and omega, see RKJ 2020-09-23 p 33
    ):
        self.s_plan = None
        self.path = None,
        self.lr_model = lr_model
        self.approach_rho = approach_rho
        self.min_rho = min_rho
        self.rhohat = rhohat

        # the offset from lr_model base of zero-vy center of rotation RKJ 2020-09-30 p 41
        (pyl, pyr, fy) = self.lr_model[1]
        (pol, por, fo) = self.lr_model[2]
        self.dwheel = (fo / fy) * (pyr - pyl) / (por - pol)

        # the rotation center in robot coordinates
        self.wheel_r = (-self.dwheel, 0.0, 0.0)
        # the robot base (measurement point) relative to the center of rotation
        self.robot_w = (self.dwheel, 0.0, 0.0)
        print(fstr({'wheel_r': self.wheel_r}))

    def start(self, start_pose, end_pose, start_twist=Twist(), end_twist=Twist(), start_t=None, motor_max = 0.9):
        self.start_pose = start_pose
        self.end_pose = end_pose
        self.start_twist = start_twist
        self.end_twist = end_twist
        self.start_t = time.time() if start_t is None else start_t
        self.motor_max = motor_max

        # convert to 2D coordinates x, y, theta. The pose coordinates are in the frame which will
        # be considered the base frame for these calculations.
        a_start = euler_from_quaternion(q_to_array(self.start_pose.orientation))[2]
        a_end = euler_from_quaternion(q_to_array(self.end_pose.orientation))[2]
        self.frame_m = (0., 0., 0.) # this is the privileged frame that others are relative to
        self.start_m = (start_pose.position.x, start_pose.position.y, a_start)
        self.end_m = (end_pose.position.x, end_pose.position.y, a_end)

        # at the start, the robot base is at start_m, so that is the origin of the robot frame
        self.wheelstart_m = transform2d(self.wheel_r, self.start_m, self.frame_m)

        # the path plan will be done in the wheel coordinates
        self.frame_p = self.wheelstart_m[:]
        self.start_p = (0., 0., 0.)

        # a frame centered on robot base is the same as the base point in m frame
        self.wheelend_m = transform2d(self.wheel_r, self.end_m, self.frame_m)
        print(fstr({'start_m': self.start_m, 'end_m': self.end_m, 'wheelstart_m': self.wheelstart_m, 'wheelend_m': self.wheelend_m}))

        self.end_p = transform2d(self.wheelend_m, self.frame_m, self.frame_p)
        print(fstr({'end_p': self.end_p}))

        # Initial Motion Plan

        # select the best two segment solution, if it exists
        paths2a = twoArcPath(*self.end_p)
        path2a = None
        for p in paths2a:
            if p[0]['radius'] > self.min_rho:
                # pylint: disable=unsubscriptable-object
                if path2a is None or (p[0]['radius'] > self.min_rho and p[0]['radius'] < path2a[0]['radius']):
                    path2a = p

        print('2 segment solution')
        if path2a:
            for s in path2a:
                print(fstr(s))

        path3a = threeSegmentPath(*self.end_p, self.approach_rho)
        print('3 segment solution')
        for s in path3a:
            print(fstr(s))

        if path2a and (
             path2a[0]['radius'] > self.min_rho and path2a[0]['radius'] < self.approach_rho
        ):
            # use the 2 segment solution
            path3a = None
        else:
            # use the 3 segment solution
            path2a = None

        self.path = path2a or path3a

        # add lprime to path plan, used by velocity
        sprime_sum = 0.0
        for seg in self.path:
            if 'radius' in seg:
                rho = seg['radius']
                seg['lprime'] = seg['length'] * (rho + self.rhohat) / rho
            else:
                seg['lprime'] = seg.length

        return self.path

    def speedPlan(self, vhat0, vhatcruise, vhatn, u=0.25):
        '''
        Given the path plan, determine a speed plan. Speeds are in 'hat' transformed form,
        see RKJ 2020-09-23 p 33

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
            # pylint: disable=unsupported-membership-test,unsubscriptable-object
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
        # return the expected position and velocities at time dt from plan start.
        # these are all in wheel coordinates

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
            lprime = seg['lprime']

            if sprime_sum + lprime > sprime:
                # this is the correct segment
                frac = (sprime - sprime_sum) / lprime
                rho = seg['radius'] if 'radius' in seg else None
                if rho is None:
                    v = vhat
                    omega = 0.0
                    theta = seg['start'][2]
                    bx = seg['start'][0] + frac * (seg['end'][0] - seg['start'][0])
                    by = seg['start'][1] + frac * (seg['end'][1] - seg['start'][1])
                else:
                    v = vhat * rho / (rho + self.rhohat)
                    omega = math.copysign(v / rho, seg['angle'])
                    arc_angle = seg['angle'] * frac
                    theta = seg['start'][2] + arc_angle
                    if arc_angle > 0.0:
                        bx = seg['center'][0] + rho * math.sin(theta)
                        by = seg['center'][1] - rho * math.cos(theta)
                    else:
                        bx = seg['center'][0] - rho * math.sin(theta)
                        by = seg['center'][1] + rho * math.cos(theta)
    
                p = (bx, by, theta)
                break
            else:
                sprime_sum += lprime
                s_sum += seg['length']

        if v is None:
            p = self.path[-1]['end']
            if 'radius' in self.path[-1]:
                rho = self.path[-1]['radius']
                v = vhat * rho / (rho + self.rhohat)
                omega = math.copysign(v / rho, self.path[-1]['angle'])
            else:
                v = vhat
                omega = 0.0
                rho = None
            
        return {'time': dt, 'sprime': sprime, 'vhat': vhat, 'v': v, 'omega': omega, 'rho': rho, 'point': p}

    def nearestPlanPoint(self, wheel_pose_p):
        # find closest point on plan

        nearests = []
        for segment in self.path:
            if 'radius' in segment and segment['radius'] is not None:
                # an arc
                fraction, nearestPoint = nearestArcPoint(
                    segment['center'],
                    segment['radius'],
                    segment['start'][2] - math.copysign(HALFPI, segment['angle']), 
                    segment['angle'],
                    wheel_pose_p[:2]
                )
            else:
                # a line
                fraction, nearestPoint = nearestLinePoint(
                    segment['start'][:2],
                    segment['end'][:2],
                    wheel_pose_p[:2])
            # note nearest
            nearestPoint_p = (
                *nearestPoint,
                (segment['start'][2] + fraction * (segment['end'][2] - segment['start'][2]))
            )
            nearests.append((fraction, nearestPoint_p))

        #print(nearests)
        best_distance = None
        best_segment = None
        best_fraction = None
        nearest_m = None
        #print(fstr({'wheel_pose_p': wheel_pose_p, 'nearests': nearests}))
        for i in range(len(nearests)):
            (fraction, nearestPoint_p) = nearests[i]
            # nearestPoint_p is where the wheels should be. The dynamic model is the position of the
            # base in the map frame. Use the nearest point as the origin of a wheels frame, and
            # determine the location of the robot base.
            nearestPoint_m = transform2d(nearestPoint_p, self.frame_p, self.frame_m)
            maybe_nearest_m = transform2d(self.robot_w, nearestPoint_m, self.frame_m)
            #print(fstr({'maybe_nearest_m': maybe_nearest_m, 'pose_m': pose_m}))
            d = math.sqrt((maybe_nearest_m[0] - pose_m[0])**2 + (maybe_nearest_m[1] - pose_m[1])**2)
            #print(d, nearestPoint, pose)
            if best_distance is None or d < best_distance:
                nearest_m = maybe_nearest_m
                best_distance = d
                best_fraction = fraction
                best_segment = i

        ### determine the speed at that point
        # Since the speed plan is in lprime, we need lprime at the point.
        sprime_sum = 0.0
        lprime = 0.0
        for i in range(len(self.path)):
            seg = self.path[i]
            if i == best_segment:
                lprime = sprime_sum + best_fraction * seg['lprime']
            else:
                sprime_sum += seg['lprime']

        # get the speed from the speed plan
        vhat = 0.0
        vsq = 0.0
        for seg in self.s_plan:
            if lprime >= seg['start'] and lprime <= seg['end']:
                if seg['time'] > 0.0:
                    vsq = seg['vstart']**2 + 2.0 * (lprime - seg['start']) * (seg['vend'] - seg['vstart']) / seg['time']
                if vsq > 0.0:
                    vhat = sqrt(vsq)
                else:
                    vhat = seg['vstart']

        seg = self.path[best_segment]
        if 'radius' in seg:
            speed = vhat * seg['radius'] / (self.rhohat + seg['radius'])
        else:
            speed = vhat
        return (nearest_m, best_distance, speed)

class DynamicStep:
    def __init__(self, lr_model=default_lr_model()):
        self.lr_model = lr_model

    def __call__(self, lr, dt, pose_m, twist_m):
        # apply lr=(left, right) over time dt, starting from pose=(x, y, theta) and twist=(vx, vy, omega)
        (x_m, y_m, theta_m) = pose_m
        (vx_m, vy_m, omega) = twist_m
        (left, right) = lr

        (pxl, pxr, fx) = self.lr_model[0]
        (pyl, pyr, fy) = self.lr_model[1]
        (pol, por, fo) = self.lr_model[2]

        # robot frame velocities
        vx_r = cos(theta_m) * vx_m + sin(theta_m) * vy_m
        vy_r = -sin(theta_m) * vx_m + cos(theta_m) * vy_m

        # dynamic model in robot coordinates
        omegaNew = omega + dt * (left * pol + right * por - fo * omega)
        vxNew_r = vx_r + dt * (left * pxl + right * pxr - fx * vx_r)
        vyNew_r = vy_r + dt * (left * pyl + right * pyr - fy * vy_r)

        # map (or base) coordinate values
        thetaNew_m = theta_m + dt * 0.5 * (omega + omegaNew)
        vxNew_m = cos(thetaNew_m) * vxNew_r - sin(thetaNew_m) * vyNew_r
        vyNew_m = sin(thetaNew_m) * vxNew_r + cos(thetaNew_m) * vyNew_r
        xNew_m = x_m + dt * 0.5 * (vx_m + vxNew_m)
        yNew_m = y_m + dt * 0.5 * (vy_m + vyNew_m)
        return ((xNew_m, yNew_m, thetaNew_m), (vxNew_m, vyNew_m, omegaNew), (vx_r, vy_r, omegaNew))

### MAIN PROGRAM ###

pp = PathPlan(approach_rho=0.20, min_rho=0.05, rhohat=0.2)
start_x = 0.0
start_y = 0.0
start_theta_degrees = 0.0
start_theta = start_theta_degrees * D_TO_R
end_theta_degrees = start_theta_degrees + 0.0
end_theta = end_theta_degrees * D_TO_R
end_x = start_x + 0.20
end_y = start_y + 0.1
start_omega = 0.0
start_vx_r = 0.20
start_vy_r = pp.dwheel * start_omega
end_vx_r = 0.0
max_t = 5.0
vhatcruise = 0.30

# transform base to plan coordinates
pose1 = Pose()
pose1.position.x = start_x
pose1.position.y = start_y
pose2 = Pose()
pose2.position.x = end_x
pose2.position.y = end_y
pose1.orientation = array_to_q(quaternion_from_euler(0.0, 0.0, start_theta))
pose2.orientation = array_to_q(quaternion_from_euler(0.0, 0.0, end_theta))

path_plan = pp.start(pose1, pose2)
print('path plan:')
for p in path_plan:
    print(fstr(p))

vhat0 = start_vx_r
vhatn = end_vx_r
s_plan = pp.speedPlan(vhat0, vhatcruise, vhatn, u=0.25)
print('speed plan:')
for s in s_plan:
    print(fstr(s))

dt = 0.05
tt = 0.0

motor = Motor()
speeds = []
while tt < max_t:
    ss = pp.v(tt)
    ss['lr'] = motor(ss['v'], ss['omega'])
    speeds.append(ss)
    if tt > ss['time'] + 5 * dt:
        break # at end of plan
    tt += dt
    print(fstr(ss))

# apply the dynamic model to test accuracy
dynamicStep = DynamicStep()

# test of control strategy
tt = 0.0

# plot the movement of robot base

pose_m = pp.start_m[:]
start_vx_m = start_vx_r * cos(start_theta) - start_vy_r * sin(start_theta)
start_vy_m = start_vx_r * sin(start_theta) + start_vy_r * cos(start_theta)
twist_m = (start_vx_m, start_vy_m, start_omega)
(left, right) = speeds[0]['lr']
(leftOld, rightOld) = motor(start_vx_r, start_omega)

ii = 0
while tt < max_t:
    ii += 1
    if ii >= len(speeds):
        break
    tt += dt

    # dynamic model
    (pose_m, twist_m, twist_r) = dynamicStep(( 0.5 * (left + leftOld), 0.5 * (right + rightOld)), dt, pose_m, twist_m)
    leftOld = left
    rightOld = right

    ### control model
    # get the wheel plan coordinates given the base coordinates. The base coordinates
    # are the origin of the robot frame, and we have the wheels position in that frame.
    wheel_pose_p = transform2d(pp.wheel_r, pose_m, pp.frame_p)
    #print(fstr({'wheel_r': pp.wheel_r, 'pose_m': pose_m, 'frame_p': pp.frame_p, 'wheel_pose_p': wheel_pose_p}))

    # find closest point on plan
    (nearest_m, distance, speed) = pp.nearestPlanPoint(wheel_pose_p)

    (left, right) = speeds[ii]['lr']
    #TODO: model is motion of wheels, we need to convert to motion of robot base
    print(fstr({
        't': tt,
        'd': distance,
        'lr': (left, right),
        'model_s': pose_m,
        'plan': speeds[ii]['point'],
        'nearest': nearest_m,
        'model_t': twist_r, 
        'plan_speed': speed}))
    #print(fstr({'best_fraction': best_fraction, 'best_segment': best_segment}))
