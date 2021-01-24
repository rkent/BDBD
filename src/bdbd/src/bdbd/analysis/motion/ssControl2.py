# state-space control of motion equations

from re import L
import numpy as np
import control
import copy
import time
import scipy
import math
from bdbd_common.utils import fstr, gstr
from bdbd_common.pathPlan2 import PathPlan
from bdbd_common.geometry import lr_est, default_lr_model, D_TO_R, transform2d, pose3to2, DynamicStep
import matplotlib.pyplot as plt
try:
    from Queue import Queue
except:
    from queue import Queue

def estr(a):
    return fstr(a, fmat='8.4g', n_per_line=10)

class PathPlot():
    def __init__(self, history_rate=5):
        self.fig = plt.figure(figsize=(12,4))
        self.base_pxj = None
        self.base_pyj = None
        self.base_frame = None
        self.history_pxwj_base = []
        self.history_pywj_base = []
        self.history_wheelx = []
        self.history_wheely = []
        self.count = 0
        self.plan_history = []
        self.history_rate = history_rate
        self.history_pxrj_base = []
        self.history_pyrj_base = []
        self.history_lefts = []
        self.history_rights = []
        self.history_tees= []
    def __call__(self, dt, source):
        self.count += 1
        s = source
        if self.base_frame is None:
            # save the first predicted trajectory and frame
            self.base_pxj = source.pxwj.copy()
            self.base_pyj = source.pywj.copy()
            self.base_frame = source.noww_pose_map
            self.base_pxrj = source.pxrj.copy()
            self.base_pyrj = source.pyrj.copy()
            #print(gstr({'base_pxj': self.base_pxj, 'base_pyj': self.base_pyj}))

        if hasattr(s, 'left') and hasattr(s, 'right') and hasattr(s, 'tee'):
            self.history_lefts.append(s.left)
            self.history_rights.append(s.right)
            self.history_tees.append(s.tee)

        fig = self.fig
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

        plt1.plot(tees, s.lefts)
        plt1.plot(tees, s.rights)
        plt1.plot(self.history_tees, self.history_lefts, 'g+')
        plt1.plot(self.history_tees, self.history_rights, 'r+')
        plt1.set_title('left,right')
        # plt1.plot(s.tees, s.omegaj)

        # Transform to base_frame coordinates
        pxj_base = []
        pyj_base = []
        for i in range(len(s.pxwj)):
            p_wheel = (s.pxwj[i], s.pywj[i], s.thetaj[i])
            p_base = transform2d(p_wheel, s.noww_pose_map, self.base_frame)
            pxj_base.append(p_base[0])
            pyj_base.append(p_base[1])

        #print(gstr({'base_frame':self.base_frame, 'pxj_base': pxj_base, 'pyj_base': pyj_base}))
        noww_pose_base = transform2d(s.noww_pose_map, (0., 0., 0.), self.base_frame)
        self.history_pxwj_base.append(noww_pose_base[0])
        self.history_pywj_base.append(noww_pose_base[1])
        nowr_pose_base = transform2d(s.nowr_pose_map, (0., 0., 0.), self.base_frame)
        self.history_pxrj_base.append(nowr_pose_base[0])
        self.history_pyrj_base.append(nowr_pose_base[1])

        #if self.history_rate and self.count % self.history_rate == 1:
        #    self.plan_history.append((pxj_base, pyj_base))
        #else:
        #    #plt2.plot(pxj_base, pyj_base)
        #    pass
        #if self.history_rate:
        #    for plan in self.plan_history:
        #        plt2.plot(plan[0], plan[1])
        plt2.plot(self.base_pxj, self.base_pyj)
        plt2.plot(self.base_pxrj, self.base_pyrj)
        #plt2.plot(pxj_base, pyj_base)
        plt2.plot(self.history_pxwj_base, self.history_pywj_base, 'r+')
        plt2.plot(self.history_pxrj_base, self.history_pyrj_base, 'g+')
        plt2.set_title('px vs py')

        #plt3.plot(tees, s.pxj)
        #plt3.plot(tees, s.pyj)
        #plt3.set_title('px,py vs t')
        plt.pause(.001)

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
    details_v=False,
    max_segments=3,
    vhat_start=0.0
):

    if details:
        print(fstr({
            's_pose': start_pose,
            't_pose': target_pose,
            's_twist': start_twist,
            't_twist': target_twist
        }))
    # estimate left, right to achieve the path
    pp = PathPlan(approach_rho=approach_rho, min_rho=min_rho)

    pathPlan = pp.start2(start_pose, target_pose, max_segments=max_segments)

    #print(dt, start_twist[0], cruise_v, target_twist[0], u)
    # calculate start vhat from start_twist
    if vhat_start is None:
        vhat_start = abs(start_twist[0]) + abs(start_twist[2] * pp.rhohat)
    speedPlan = pp.speedPlan(vhat_start, cruise_v, target_twist[0], u=u)

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
    poses = [(0., 0., 0.)]
    vv = None
    while tt < total_time:
        tt += dt
        vv = pp.v(tt)
        vvs.append(vv)
        # vv gives vhat is in wheel frame. We need to convert to robot frame.
        vxres.append(vv['v'])
        vyres.append(vv['omega'] * pp.dwheel)
        omegas.append(vv['omega'])

        if details_v:
            print(fstr(vv))
        (left, right, last_vx, last_omega) = lr_est(
            vv['v'], vv['omega'], last_vx, last_omega, dt,
            lr_model=lr_model, mmax=mmax)
        lefts.append(left)
        rights.append(right)
        tees.append(tt)
        poses.append(vv['point'])
        vv['left'] = left
        vv['right'] = right

    if details:
        if vv:
            print('vv at end:' + fstr(vv))
        print('pathPlan:')
        for segment in pathPlan:
            print(fstr(segment, fmat='7.4f'))
        print('speedPlan')
        for segment in speedPlan:
            print(fstr(segment, fmat='7.4f'))
    num_segments = len(pathPlan)
    return (tees, lefts, rights, num_segments, pp, total_time, vxres, omegas, poses)

if __name__ == '__main__':

    dt = 0.02
    start_pose = [0.0, 0.0, 0.0]
    start_twist = [0.0, 0.0, 0.0]
    target_pose = [.26, .52, -D_TO_R * 180]
    target_twist = [0.0, 0.0, 0.0]
    approach_rho = 0.30
    min_rho = 0.05
    cruise_v = 0.25
    lr_start = (0.0, 0.0)
    mmax = 1.0
    u_time = 0.50
    Qfact = [1, 1, 1, 1, 1, 1]
    lr_model = default_lr_model()
    # scale vy to match omega timing
    #for i in range(3):
    #    lr_model[1][i] = lr_model[1][i] * lr_model[2][2] / lr_model[1][2]
    print(gstr({'lr_model': lr_model}))
    bad_lr_model = copy.deepcopy(lr_model)

    bad_lr_model[0][1] *= .9
    bad_lr_model[0][0] *= 1.0
    bad_lr_model[2][1] *= 1.1
    bad_lr_model[2][0] *= 1.0

    # poles for state control model
    fact = 0.90
    base = -2.0
    poles = []
    for i in range(6):
        poles.append(base)
        base = base * fact
    np_poles = np.array(poles)

    dynamicStep = DynamicStep(bad_lr_model)

    #lr_model = ((1.0, 1.0, 10.0), (-1.0, 1.0, 10.0), (-1.0, 10.0, 10.0))
    (bxl, bxr, qx) = lr_model[0]
    (bol, bor, qo)= lr_model[2]

    B = np.array((
            (bxl, bxr),
            (0, 0),
            (bol, bor),
            (0, 0),
            (0, 0),
            (0, 0)
    ))

    zero3 = (0.0, 0.0, 0.0)
 
    pathPlot = PathPlot(history_rate=1000)
    (tees, lefts, rights, max_segments, pp, plan_time, vxres, omegas, poses) = static_plan(dt
        , start_pose=zero3
        , start_twist=zero3
        , target_pose=target_pose
        , target_twist=target_twist
        , approach_rho=approach_rho
        , min_rho=min_rho
        , cruise_v=cruise_v
        , mmax=mmax
        , u=u_time
        , details=True
        , vhat_start=0.0
    )
    class Object():
        pass
    source = Object()
    source.pxwj = []
    source.pywj = []
    source.pxrj = []
    source.pyrj = []
    source.thetaj = []
    source.tees = tees
    source.lefts = lefts
    source.rights = rights
    # the base frame (zero at start of plan) is the wheel location
    frame_m = zero3
    source.noww_pose_map = zero3
    source.nowr_pose_map = pp.robot_w
    for pose in poses:
        wheel_p = pose
        wheel_m = pose
        robot_m = transform2d(pp.robot_w, wheel_m, frame_m)
        source.pxwj.append(wheel_m[0])
        source.pywj.append(wheel_m[1])
        source.thetaj.append(wheel_m[2])
        source.pxrj.append(robot_m[0])
        source.pyrj.append(robot_m[1])
    pathPlot(dt, source)

    # now use the dynamic/control model
    pose_m = source.nowr_pose_map
    twist_m = zero3
    next_left = lefts[0]
    next_right = rights[0]

    '''
    Q = np.identity(6)
    for i in range(len(Qfact)):
        Q[i][i] *= Qfact[i]
    R = 1.0 * np.identity(2)
    '''
    stepCount = 0
    #for i in range(len(tees)):

    # initial step
    vv = pp.v(0.0)
    print(fstr({'initial_vv': vv}))
    omega0 = vv['omega']
    s0 = vv['v']
    # Note that "rho" is always positive, but "kappa" has the same sign as omega.
    if vv['kappa'] is None:
        sdot = 0.0
        odot = vv['direction'] * vv['d_vhat_dt'] / pp.rhohat
    else:
        sdot = vv['d_vhat_dt'] / (1.0 + pp.rhohat * abs(vv['kappa']))
        odot = vv['kappa'] * sdot
    corr_left = 0.0
    corr_right = 0.0
    
    tt = 0.0
    while True:
        print(' ')
        stepCount += 1
        tt += dt
        '''
        if stepCount % 1000 == 10000:
            (tees, lefts, rights, max_segments, pp, plan_time, vxres, omegas, poses) = static_plan(dt
                , start_pose=pose_m
                , start_twist=twist_m
                , target_pose=target_pose
                , target_twist=target_twist
                , approach_rho=approach_rho
                , min_rho=min_rho
                , cruise_v=cruise_v
                , mmax=mmax
                , u=u_time
                , details=True
                , vhat_start=None
            )
        '''

        R0 = (
                (bol * (sdot + qx * s0) - bxl * (odot + qo * omega0)) /
                (bol * bxr - bxl * bor)
        )
        L0 = (
                (bor * (sdot + qx * s0) - bxr * (odot + qo * omega0)) /
                (bor * bxl - bxr * bol)
        )

        #corr_left = 0.0
        #corr_right = 0.0
        ctl_left = float(corr_left + L0)
        ctl_right = float(corr_right + R0)
        # correct for limits
        maxctl = max(abs(ctl_left), abs(ctl_right))
        if maxctl > mmax:
            ctl_left *= mmax / maxctl
            ctl_right *= mmax / maxctl
        print(fstr({'tt': tt, 'L0': L0, 'R0': R0, 'corr_left': corr_left, 'corr_right': corr_right, 'ctl_left': ctl_left, 'ctl_right': ctl_right, 'sdot': sdot, 'odot': odot, 'maxctl': maxctl}))

        (pose_m, twist_m, twist_r) = dynamicStep((ctl_left, ctl_right), dt, pose_m, twist_m)
        wheel_m = transform2d(pp.wheel_r, pose_m, zero3)

        if stepCount % 5 == 1:
            source.nowr_pose_map = pose_m
            source.noww_pose_map = wheel_m
            source.tee = tt
            source.left = ctl_left
            source.right = ctl_right
            pathPlot(dt, source)

        print(fstr({'pose_m':pose_m, 'wheel_m': wheel_m, 'twist_m': twist_m, 'twist_r': twist_r}, fmat='8.5f'))

        # control algorithm
        vv = pp.nextPlanPoint(dt, pose_m)
        print(fstr({'vv': vv}))
        if vv['fraction'] > 0.999:
            break

        # vv determines the normed frame for linearized control. Current pose
        # must be represented in this frame for control calculations. The frame definition
        # is the vv wheels in map frame
        frame_n = vv['point']
        wheel_n = transform2d(wheel_m, zero3, frame_n)
        theta_n = wheel_n[2]
        print(fstr({'frame_n': frame_n, 'pose_n': wheel_n}))

        # twist in robot frame
        vxr_r = twist_r[0]
        vyr_r = twist_r[1]
        omega_r = twist_r[2]

        # robot (measurement point) twist in normed (vv) coordinates
        vxr_n = vxr_r * math.cos(theta_n) - vyr_r * math.sin(theta_n)
        vyr_n = vxr_r * math.sin(theta_n) + vyr_r * math.cos(theta_n)
        # wheel
        vxr_w = vxr_n
        vyw_n = vyr_n - pp.dwheel * omega_r

        '''
        vv_vx_n = vv['v'] * math.cos(vv_theta_n)
        vv_vy_n = vv['v'] * math.sin(vv_theta_n)

        # vy in wheel coordinates. RKJ 2021-01-19
        vy_w = twist_r[1] - pp.dwheel * twist_r[2]
        '''
        rho = twist_r[0] / twist_r[2] if twist_r[2] != 0.0 else None

        eps = np.array([
            [vxr_w - vv['v']],
            [vyw_n],
            [omega_r - vv['omega']],
            [wheel_n[0]],
            [wheel_n[1]],
            [wheel_n[2]]
        ])
        print(fstr({'eps': eps, 'rho': rho, 'vxw_n': vyw_n}))

        omega0 = vv['omega']
        s0 = vv['v']
        # Note that "rho" is always positive, but "kappa" has the same sign as omega.
        if vv['kappa'] is None:
            sdot = 0.0
            odot = vv['direction'] * vv['d_vhat_dt'] / pp.rhohat
        else:
            sdot = vv['d_vhat_dt'] / (1.0 + pp.rhohat * abs(vv['kappa']))
            odot = vv['kappa'] * sdot

        s0 = s0 if s0 else 0.001
        A = np.array((
                (-qx, 0, 0, 0, 0, -omega0 * s0),
                (omega0, 0, s0, 0, 0, -qx * s0),
                (0, 0, -qo, 0, 0, 0),
                (1, 0, 0, 0, 0, 0),
                (0, 1, 0, 0, 0, 0),
                (0, 0, 1, 0, 0, 0)
        ))
        #print(gstr({'A': A, 'B': B}))
        #Kr, S, E = control.lqr(A, B, Q, R)
        Kr = control.place_varga(A, B, np_poles)
        # debug
        '''
        Kr = np.array([
            [.191, 3.32, -.445, .592, .806, -1.53],
            [.15, -2.65, .433, .806, -5.92, 1.37]
        ])
        Kr = np.matrix([
            [-1.67580,   0.75032,   0.17497,   0.85075,   0.26784,  -1.03952],
            [-1.72772,  -0.63308,  -0.15645,   1.35021,  -0.20051,   0.87364]
        ])
        '''
        #print(gstr({'Kr': Kr}))
        #print(gstr({'eps * Kr': np.squeeze(eps) * np.asarray(Kr)}))
        lrs = -Kr * eps
        print({'lrs': np.squeeze(lrs)})
        # RKJ 2021-01-12 p 72
        corr_left = lrs[0][0]
        corr_right = lrs[1][0]

    print('all done')
    plt.waitforbuttonpress()
    #print('!', tees)
    #print(fstr((tees, lefts, rights, max_segments, pp, plan_time, vxres, omegas, poses)))
