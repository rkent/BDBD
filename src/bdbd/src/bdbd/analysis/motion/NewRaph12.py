# newton-raphson iteration of motion equations
# 12: add option to use static path if newton-raphson fails

import numpy as np
import time
import rospy
import math
import threading
from bdbd_common.utils import fstr, gstr
from bdbd_common.geometry import lr_est, default_lr_model, D_TO_R, transform2d, pose3to2
from bdbd_common.NewRaph import NewRaph
from bdbd_common.msg import LrOptimizeGoal, LrOptimizeAction, LrResults, MotorsRaw

from nav_msgs.msg import Odometry
import actionlib
from actionlib_msgs.msg import GoalStatus
from bdbd_common.pathPlan2 import PathPlan
import matplotlib.pyplot as plt
try:
    from Queue import Queue
except:
    from queue import Queue

def nextLR(
        dt,
        callback_dt,
        last_lr,
        now_twist_robot,
        sum_epsv,
        sum_epso,
        pp,
        lr_model=default_lr_model(),
        mmax=1.0
    ):

    # Calculate next left, right from velocity model with integral error correction
    tt = 0.0
    tees = [tt]
    last_vx = now_twist_robot[0] - sum_epsv
    last_omega = now_twist_robot[2] - sum_epso
    vxes = [last_vx]
    omegas = [last_omega]
    lefts = [last_lr[0]]
    rights = [last_lr[1]]

    while True:
        tt += dt
        vv = pp.v(tt)
        v = vv['v'] - sum_epsv
        omega = vv['omega'] - sum_epso
        vxes.append(v)
        omegas.append(omega)
        print(fstr(vv))

        (left, right, last_vx, last_omega) = lr_est(
            v, omega, last_vx, last_omega, dt,
            lr_model=lr_model, mmax=mmax)
        lefts.append(left)
        rights.append(right)
        if tt > callback_dt:
            break
    new_left = average(callback_dt, tees, lefts)
    new_right = average(callback_dt, tees, rights)
    print(gstr((new_left, new_right)))
    return (new_left, new_right)

class Odom():
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/t265/odom/sample', Odometry, self.odom_cb, tcp_nodelay=True)
        self.poses = [0.0, 0.0, 0.0]
        self.twists = [0.0, 0.0, 0.0]
        self.count = 0
        self.queue = Queue()
        # start averaging data, driven by odometry message receipt
        self.odom_sub = rospy.Subscriber('/t265/odom/sample', Odometry, self.odom_cb, tcp_nodelay=True)

    def odom_cb(self, odometry):
        if self.queue.empty():
            self.queue.put(odometry)
    
    def process(self):
        if self.queue.empty():
            return
        odometry = self.queue.get()

        new_factor = 0.10
        pose_m = pose3to2(odometry.pose.pose)
        twist3 = odometry.twist.twist
        twist_r = (twist3.linear.x, twist3.linear.y, twist3.angular.z)
        for i in range(3):
            self.poses[i] = (1. - new_factor) * self.poses[i] + new_factor * pose_m[i]
            self.twists[i] = (1. - new_factor) * self.twists[i] + new_factor * twist_r[i]
        self.count += 1
        # rospy.loginfo('vx, vy, omega: ' + fstr(twist_r))

    def get_odom(self):
        poses = self.poses[:]
        twists = self.twists[:]
        return (poses, twists)

    def get_count(self):
        result = self.count
        return result

def static_plan(dt,
    start_pose=(0.0, 0.0, 0.0),
    start_twist=(0.0, 0.0, 0.0),
    target_pose=(0.0, 0.0, 0.0),
    target_twist=(0.0, 0.0, 0.0),
    lr_model=default_lr_model(),
    mmax=1.0,
    approach_rho=0.08, # radius of planned approach
    min_rho=0.02, # the smallest radius we allow in a path plan,
    cruise_v = 0.25,
    lr_start = (0.0, 0.0),
    u=0.50,
    details=False,
    max_segments=3,
    vhat_start=0.0
):

    if details:
        print(gstr({'start_pose': start_pose, 'target_pose': target_pose}))
    # estimate left, right to achieve the path
    pp = PathPlan(approach_rho=approach_rho, min_rho=min_rho)

    pathPlan = pp.start2(start_pose, target_pose, max_segments=max_segments)

    #print(dt, start_twist[0], cruise_v, target_twist[0], u)
    speedPlan = pp.speedPlan(vhat_start, cruise_v, target_twist[0], u=u)
    for seg in speedPlan:
        print(gstr(seg))

    total_time = 0.0
    for seg in speedPlan:
        total_time += seg['time']

    vxr0 = start_twist[0] * math.cos(start_pose[2]) + start_twist[1] * math.sin(start_pose[2])
    vyr0 = -start_twist[0] * math.sin(start_pose[2]) + start_twist[1] * math.cos(start_pose[2])
    last_vx = vxr0
    last_omega = start_twist[2]

    vxres = [vxr0]
    vyres = [vyr0]
    omegas = [start_twist[2]]
    vvs = [pp.v(0.0)]
    vvs[0]['left'] = lr_start[0]
    vvs[0]['right'] = lr_start[1]
    lefts = [lr_start[0]]
    rights = [lr_start[1]]
    tt = 0.0
    tees = [tt]
    while tt < total_time:
        tt += dt
        vv = pp.v(tt)
        vvs.append(vv)
        # vv gives vhat is in wheel frame. We need to convert to robot frame.
        vxres.append(vv['v'])
        vyres.append(vv['omega'] * pp.dwheel)
        omegas.append(vv['omega'])

        if tt < 0.10:
            print("tt, vv['v'], vv['omega'], last_vx, last_omega: " + fstr((tt, vv['v'], vv['omega'], last_vx, last_omega)))
        (left, right, last_vx, last_omega) = lr_est(
            vv['v'], vv['omega'], last_vx, last_omega, dt,
            lr_model=lr_model, mmax=mmax)
        #if tt < 0.10:
        #    print("(left, right, last_vx, last_omega):" + fstr((left, right, last_vx, last_omega)))
        lefts.append(left)
        rights.append(right)
        tees.append(tt)
        vv['left'] = left
        vv['right'] = right

    if details:
        print('vv:' + gstr(vv))
        print('pathPlan:')
        for segment in pathPlan:
            print(fstr(segment, fmat='7.4f'))
        print('speedPlan')
        for segment in speedPlan:
            print(fstr(segment, fmat='7.4f'))
    num_segments = len(pathPlan)
    return (tees, lefts, rights, num_segments, pp, total_time, vxres, omegas)

def poses(dt, ls, rs,
    start_pose=(0.0, 0.0, 0.0),
    start_twist=(0.0, 0.0, 0.0),
    lr_model=default_lr_model()
):
    als = np.asarray(ls)
    ars = np.asarray(rs)
    # prep constants for calculations
    alr_model = np.array(lr_model)
    bhes = (dt * alr_model[0], dt * alr_model[1], dt * alr_model[2])
    (bhxl, bhxr, qhx) = bhes[0]
    (bhyl, bhyr, qhy) = bhes[1]
    (bhol, bhor, qho) = bhes[2]
    (alphax, alphay, alphao) = 1.0 - np.array((qhx, qhy, qho))
    n = len(ls)

    # alpha ** j
    alphaxj = [1.0]
    alphayj = [1.0]
    alphaoj = [1.0]
    betaj = [dt]
    for i in range(1, n):
        alphaxj.append(alphaxj[i-1] * alphax)
        alphayj.append(alphayj[i-1] * alphay)
        alphaoj.append(alphaoj[i-1] * alphao)
        betaj.append(betaj[i-1] + dt * alphaoj[i])

    #print('als:' + estr(als))
    (px0, py0, theta0) = start_pose
    (vxw0, vyw0, omega0) = start_twist

    # initial robot velocities
    vx0 = vxw0 * math.cos(theta0) + vyw0 * math.cos(theta0)
    vy0 = -vxw0 * math.sin(theta0) + vyw0 * math.cos(theta0)

    # twist
    vxj = np.empty(n)
    vyj = np.empty(n)
    omegaj = np.empty(n)
    vxj[0] = vx0
    vyj[0] = vy0
    omegaj[0] = omega0
    bmotorxj = bhxl * als + bhxr * ars
    bmotoryj = bhyl * als + bhyr * ars
    bmotoroj = bhol * als + bhor * ars
    for i in range(1, n):
        vxj[i] = vx0 * alphaxj[i] + np.dot(alphaxj[i-1::-1], bmotorxj[1:i+1])
        vyj[i] = vy0 * alphayj[i] + np.dot(alphayj[i-1::-1], bmotoryj[1:i+1])
        omegaj[i] = omega0 * alphaoj[i] + np.dot(alphaoj[i-1::-1], bmotoroj[1:i+1])

    # pose
    pxj = np.empty(n)
    pyj = np.empty(n)
    thetaj = np.empty(n)
    pxj[0] = px0
    pyj[0] = py0
    thetaj[0] = theta0
    for i in range(1, n):
        thetaj[i] = theta0 + omega0 * (betaj[i] - dt) \
            + np.dot(betaj[i-1::-1], bmotoroj[1:i+1])

    # intermediate values as vectors
    cosj = np.cos(thetaj)
    sinj = np.sin(thetaj)
    vxcj = vxj * cosj
    vxsj = vxj * sinj
    vycj = vyj * cosj
    vysj = vyj * sinj
    vxwj = vxcj - vysj
    vywj = vxsj + vycj

    pxj[1:] = px0 + dt * np.cumsum(vxwj[1:])
    pyj[1:] = py0 + dt * np.cumsum(vywj[1:])

    return (pxj, pyj, thetaj, vxj, vyj, omegaj)

def estr(a):
    return fstr(a, fmat='8.4g', n_per_line=10)

def average(dt, tees, m):
    # average of m from 0 to dt
    m_sum = 0.0
    for i in range(1, len(tees)):
        interval_dt = tees[i] - tees[i-1] if tees[i] < dt else dt - tees[i-1]
        m_sum += (m[i] + m[i-1]) * interval_dt
        if dt < tees[i]:
            break
    return 0.5 * m_sum / dt

class LrOptimizeClient:
    def __init__(self,
        Wmax=1.0e-2,
        Wjerk=1.0e-2,
        Wback=1.0,
        mmax=1.0,
        queue=Queue()
    ):
        rospy.loginfo('init LrOptimizeClient')
        self.Wmax = Wmax
        self.Wjerk = Wjerk
        self.Wback = Wback
        self.mmax = mmax
        self.queue = queue
        self.goal = None

    def done_cb(self, state, r):
        rospy.loginfo(gstr({'done state': GoalStatus.to_string(state), 'elapsed time': time.time() - self.start, 'loss': r.loss}))
        if state == GoalStatus.SUCCEEDED:
            self.queue.put(r)

    def feedback_cb(self, feedback):
        if self.queue.empty():
            self.queue.put(feedback)
        pass

    def __call__(self, dt, lefts, rights
        ,gaussItersMax=0
        ,nrItersMax=1
        ,start_pose=(0.0, 0.0, 0.0)
        ,start_twist=(0.0, 0.0, 0.0)
        ,target_pose=(0.0, 0.0, 0.0)
        ,target_twist=(0.0, 0.0, 0.0)
        ,target_lr=(0.0, 0.0)
        ,rate=20

    ):
        self.start = time.time()
        self.client = actionlib.SimpleActionClient('/bdbd/dynamicPath', LrOptimizeAction)
        self.client.wait_for_server()
        self.feedback_sub = rospy.Subscriber('/bdbd/dynamicPath/feedsub', LrResults, self.feedback_cb)

        goal = LrOptimizeGoal()
        goal.dt = dt
        goal.start_lefts = lefts
        goal.start_rights = rights
        goal.start_pose = start_pose
        goal.start_twist = start_twist
        goal.target_pose = target_pose
        goal.target_twist = target_twist
        goal.target_lr = target_lr
        goal.Wmax = self.Wmax
        goal.Wjerk = self.Wjerk
        goal.Wback = self.Wback
        goal.mmax = self.mmax
        goal.gaussItersMax = gaussItersMax
        goal.nrItersMax = nrItersMax
        goal.rate = rate
        self.goal = goal
        self.client.send_goal(goal, done_cb=self.done_cb, feedback_cb=self.feedback_cb)
        rospy.loginfo('waiting for result')

class PathPlot():
    def __init__(self, history_rate=5):
        self.fig = plt.figure(figsize=(12,4))
        self.base_pxj = None
        self.base_pyj = None
        self.base_frame = None
        self.history_pxj_base = []
        self.history_pyj_base = []
        self.history_wheelx = []
        self.history_wheely = []
        self.count = 0
        self.plan_history = []
        self.history_rate = history_rate
    def __call__(self, dt, source):
        self.count += 1
        if self.base_frame is None:
            # save the first predicted trajectory and frame
            self.base_pxj = source.pxj.copy()
            self.base_pyj = source.pyj.copy()
            self.base_frame = source.now_pose_map

        fig = self.fig
        s = source
        if hasattr(s, 'tees'):
            tees = s.tees
        else:
            tees = [0.0]
            tt = 0.0
            for i in range(1, len(s.lefts)):
                tt += dt
                tees.append(tt)

        fig.clf()
        plt1 = fig.add_subplot(131)
        #plt1.axis([0.0, tfPath.lrs[-1]['t'], -1.5, 1.5])
        plt2 = fig.add_subplot(132)
        plt3 = fig.add_subplot(133)

        #if axis3 is not None:
        #    plt3.axis(axis3)

        plt2.axis('equal')

        #plt1.plot(tees, s.lefts)
        #plt1.plot(tees, s.rights)
        #plt1.set_title('left,right')
        # plt1.plot(s.tees, s.omegaj)

        # Transform to base_frame coordinates
        pxj_base = []
        pyj_base = []
        for i in range(len(s.pxj)):
            p_robot = (s.pxj[i], s.pyj[i], s.thetaj[i])
            p_base = transform2d(p_robot, s.now_pose_map, self.base_frame)
            pxj_base.append(p_base[0])
            pyj_base.append(p_base[1])

        now_pose_base = transform2d(s.now_pose_map, (0., 0., 0.), self.base_frame)
        self.history_pxj_base.append(now_pose_base[0])
        self.history_pyj_base.append(now_pose_base[1])
        self.history_wheelx.append(s.now_pose_wheel[0])
        self.history_wheely.append(s.now_pose_wheel[1])

        if self.count % self.history_rate == 0:
            self.plan_history.append((pxj_base, pyj_base))
        else:
            plt2.plot(pxj_base, pyj_base)
        for plan in self.plan_history:
            plt2.plot(plan[0], plan[1])
        #plt2.plot(self.base_pxj, self.base_pyj)
        plt2.plot(pxj_base, pyj_base)
        plt2.plot(self.history_pxj_base, self.history_pyj_base, 'r+')
        #plt2.plot(self.history_wheelx, self.history_wheely, 'g+')
        plt2.set_title('px vs py')

        #plt3.plot(tees, s.pxj)
        #plt3.plot(tees, s.pyj)
        #plt3.set_title('px,py vs t')
        plt.pause(.001)

if __name__ == '__main__':

    dt = 0.02
    lr_model = default_lr_model()
    #lr_model = ((1.0, 1.0, 10.0), (-1.0, 1.0, 10.0), (-1.0, 10.0, 10.0))
    start_pose = [0.0, 0.0, 0.0]
    start_twist = [0.0, 0.0, 0.0]
    target_pose = [0.30, 0.0, D_TO_R * 45]
    target_twist = [0.0, 0.0, 0.0]
    approach_rho = 0.15
    min_rho = 0.10
    cruise_v = 0.30
    lr_start = (0.0, 0.0)
    gauss_iters = 20
    nr_iters = 20
    Wmax = 1.e-6
    #Wmax = 0.0
    Wjerk = 1.e-2
    Wback = 1.0
    #Wback = 0.0
    mmax = 1.0
    NRstart = 1.0
    NRfact = 2
    maxSlew = 2.00
    testNR = False
    maxN = 50
    rate = 20
    callback_dt = 1. / rate
    min_lr = 0.15
    MAX_STEPS=10
    integralK = 5 * dt

    rospy.init_node('NewRaph', log_level=rospy.DEBUG)
    odom = Odom()
    while odom.get_count() < 20:
        odom.process()
        rospy.sleep(0.001)

    zero3 = (0.0, 0.0, 0.0)
    (start_pose_map, start_twist_robot) = odom.get_odom()
    target_pose_map = transform2d(target_pose, start_pose_map, zero3)
    #print(gstr({'start_pose_map': start_pose_map, '\nstart_twist_robot': start_twist_robot, '\ntarget_pose_map': target_pose_map}))

    # Main Loop
    rosrate = rospy.Rate(20)
    first_time = True
    now_pose_old = zero3
    progress_dt = 0.0
    motor_pub = rospy.Publisher('/bdbd/motors/cmd_raw', MotorsRaw, queue_size=10)
    pathPlot = PathPlot(history_rate=5)

    (tees, lefts, rights, max_segments, pp, plan_time, vxres, omegas) = static_plan(dt
        , start_pose=zero3
        , start_twist=start_twist_robot
        , target_pose=target_pose
        , target_twist=target_twist
        , approach_rho=approach_rho
        , min_rho=min_rho
        , cruise_v=cruise_v
        , mmax=mmax
        , details=False
        , vhat_start=0.0
    )
    vhat_start = pp.v(callback_dt)['vhat']

    (pxj, pyj, thetaj, vxj, vyj, omegaj) = poses(dt, lefts, rights)
    n = len(lefts)
    total_time = dt * (n - 1)
    print('Entering main loop, total_time', total_time)
    #while not rospy.is_shutdown():
    #    rospy.sleep(.001)
    last_lr = (0.0, 0.0)
    new_leftp1 = None
    new_rightp1 = None
    new_vp1 = 0.0
    new_op1 = 0.0
    sum_epsv = 0.0
    sum_epso = 0.0

    last_time = time.time()
    count = 0
    while not rospy.is_shutdown():
        count += 1
        if count > MAX_STEPS:
            print('MAX_STEPS reached')
            break
        now_time = time.time()
        times = [now_time]
        next_time = last_time + callback_dt
        last_time = now_time
        rospy.sleep(0.001)
        times.append(time.time())
        #plt.pause(0.001)
        #times.append(time.time())
        odom.process()
        times.append(time.time())
        while time.time() < next_time and not rospy.is_shutdown():
            odom.process()
        times.append(time.time())
        (now_pose_map, now_twist_robot) = odom.get_odom()
        now_pose_start = transform2d(now_pose_map, zero3, start_pose_map)
        now_pose_wheel = transform2d(pp.wheel_r, now_pose_start, start_pose_map)
        target_pose = transform2d(target_pose_map, zero3, now_pose_map)
        #if max_segments == 1 and target_pose[0] < 0.0:
        #    break

        if not first_time:
            first_time = False
            last_distance_sq = 1.e10
            i = 0
            for i in range(n):
                dx2 = (now_pose_old[0] - pxj[i])**2
                dy2 = (now_pose_old[1] - pyj[i])**2
                # adjust theta error using RKJ 2020-12-7
                # TODO: the following makes no sense, see equivalent in dynamicPath
                dtheta2 = (min_rho * (1.0 - math.cos(now_pose_old[2], thetaj[i])))**2
                distance_sq = dx2 + dy2 + dtheta2
                d0 = d1
                d1 = d2
                d2 = distance_sq
                if (distance_sq > last_distance_sq and i >= 2):
                    break
                last_distance_sq = distance_sq
            denom = 2. * (d0 + d2 - 2 * d1)
            closest_i = i - 1 if denom == 0.0 else (i-2) + (3.*d0 + d2 - 4.*d1) / denom
            closest_i = min(i, max(i-2, closest_i))
            progress_dt = max(callback_dt / 4., dt * closest_i)

        times.append(time.time())
        total_time -= progress_dt
        if total_time < callback_dt:
            print('total_time reached')
            break
        dt = total_time / (n-1)

        (tees, lefts, rights, max_segments, pp, plan_time, vxres, omegas) = static_plan(dt
            , start_pose=zero3
            , start_twist=now_twist_robot
            , target_pose=target_pose
            , target_twist=target_twist
            , approach_rho=approach_rho
            , min_rho=min_rho
            , cruise_v=cruise_v
            , mmax=mmax
            , lr_start=last_lr
            , details=False
            , max_segments=max_segments
            , vhat_start = vhat_start
        )
        vhat_start = pp.v(callback_dt)['vhat']
        vhat_now = math.sqrt(pow(now_twist_robot[0], 2) + pow(now_twist_robot[1], 2)) + pp.rhohat * abs(now_twist_robot[2])
        times.append(time.time())

        # Integral control term for v and omega

        epsv = now_twist_robot[0] - new_vp1
        epso = now_twist_robot[2] - new_op1
        sum_epsv += integralK * epsv
        sum_epso += integralK * epso

        (new_leftx, new_rightx) = nextLR(dt, callback_dt, last_lr,
            now_twist_robot, sum_epsv, sum_epso, pp, lr_model=lr_model,
            mmax=mmax)
        print(fstr((
            dt,
            callback_dt,
            last_lr,
            now_twist_robot,
            sum_epsv,
            sum_epso
        )))
        # model predictions
        new_v = average(callback_dt, tees, vxres)
        new_o = average(callback_dt, tees, omegas)
        new_v2 = average(2*callback_dt, tees, vxres)
        new_o2 = average(2*callback_dt, tees, omegas)

        new_left1 = average(callback_dt, tees, lefts)
        new_right1 = average(callback_dt, tees, rights)

        # damping of left, right to spread error correction over time
        nf = 0.25
        new_left = new_left1 if new_leftp1 is None else nf * new_left1 + (1.-nf) * new_leftp1
        new_right = new_right1 if new_rightp1 is None else nf * new_right1 + (1.-nf) * new_rightp1
        new_left2 = average(2 * callback_dt, tees, lefts)
        new_right2 = average(2 * callback_dt, tees, rights)
        new_leftp1 = 2 * new_left2 - new_left1
        new_rightp1 = 2* new_right2 - new_right1

        print(fstr({
            'new_left': new_left,
            'new_right': new_right,
            'new_left1': new_left1,
            'new_right1': new_right1,
            'new_leftp1': new_leftp1,
            'new_rightp1': new_rightp1,
            'new_leftx': new_leftx,
            'new_right': new_rightx
        }))

        print(fstr({
            'max_segments': max_segments,
            'p(x,y)_m': now_pose_start[:2],
            'theta': now_pose_start[2]/D_TO_R,
            'target_x_r': target_pose[0],
            'progress_dt': progress_dt,
            'vhat': vhat_now
        }, fmat='7.4f'))
        print(fstr({
            'twist': now_twist_robot,
            'now_pose_wheel': now_pose_wheel,
            'lefts[:3]': lefts[:3],
            'rights[:3]': rights[:3]
        }))

        (pxj, pyj, thetaj, vxj, vyj, omegaj) = poses(dt, lefts, rights)
        times.append(time.time())

        avg_lr = 0.5 * math.sqrt(new_left * new_left + new_right * new_right)
        
        #if avg_lr < min_lr:
        #    print('min_lr reached')
        #    break
        rospy.loginfo('publishing left, right (%s,%s)', new_left, new_right)
        motor_pub.publish(new_left, new_right)
        times.append(time.time())
        last_lr = (new_left, new_right)
        
        class Object():
            pass
        source = Object()
        source.tees = tees
        source.pxj = pxj
        source.pyj = pyj
        source.thetaj = thetaj
        source.lefts = lefts
        source.rights = rights
        source.now_pose_map = now_pose_map
        source.now_pose_wheel = now_pose_wheel
        #pathPlot(dt, source)
        times.append(time.time())

        deltas = []
        for i in range(1, len(times)):
            deltas.append(times[i] - times[i-1])
        print('deltas:' + gstr(deltas) + ' total:' + gstr(times[-1] - times[0]))
    print('rospy shut down')
    motor_pub.publish(0.0, 0.0)
    odom.odom_sub.unregister()
    rospy.sleep(.1)
    plt.waitforbuttonpress()
    exit(0)
    
    pathPlot = PathPlot(history_rate=3)

    # send to C++ node for processing
    lroClient = LrOptimizeClient(Wmax=Wmax, Wjerk=Wjerk, Wback=Wback, mmax=mmax)

    lroClient(dt, lefts, rights
        , gaussItersMax=gauss_iters
        , nrItersMax=nr_iters
        , start_pose=start_pose
        , start_twist=start_twist
        , target_pose=target_pose
        , rate=rate
        )
    print('\n***** started action *****')

    gauss_count = 0
    nr_count = 0
    eps = 1.0
    while not rospy.is_shutdown():
        rospy.sleep(0.001)
        plt.pause(.001)
        if not lroClient.queue.empty():
            result = lroClient.queue.get();
            '''
            tt = 0.0
            for i in range(len(result.lefts)):
                print(fstr({
                    't': tt,
                    'lr': (result.lefts[i], result.rights[i]),
                    'pose': (result.pxs[i], result.pys[i], result.thetas[i]),
                    'twist': (result.vxs[i], result.vys[i], result.omegas[i])
                }))
                tt += dt
            nr.poses(result.lefts, result.rights)
            '''
            # print(gstr({'lefts': result.lefts, 'rights': result.rights}))
            #print(lroClient.client.get_state(), lroClient.client.get_goal_status_text())
            try:
                pathPlot(result.dt, result)
                print(fstr({'n': len(result.lefts), 'x': result.now_pose_start[0], 'y': result.now_pose_start[1], 'theta deg': result.now_pose_start[2]/D_TO_R}))
            except:
                print('Error in pathPlot')
