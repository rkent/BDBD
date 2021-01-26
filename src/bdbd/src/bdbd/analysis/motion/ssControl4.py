# state-space control of motion equations

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

import math
from bdbd_common.utils import fstr, gstr
from bdbd_common.pathPlan2 import PathPlan
from bdbd_common.ssControl import SsControl
from bdbd_common.geometry import lr_est, default_lr_model, D_TO_R, transform2d, DynamicStep
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
            self.base_frame = source.nowr_pose_map
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

def static_plan(
    start_pose=(0.0, 0.0, 0.0),
    start_twist=(0.0, 0.0, 0.0),
    target_pose=(0.0, 0.0, 0.0),
    target_twist=(0.0, 0.0, 0.0),
    approach_rho=0.08, # radius of planned approach
    min_rho=0.02, # the smallest radius we allow in a path plan,
    cruise_v = 0.25,
    u=0.50,
    details=False,
    max_segments=3,
    vhat_start=None
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
    pp.start2(start_pose, target_pose, max_segments=max_segments)

    # calculate start vhat from start_twist
    if vhat_start is None:
        vhat_start = abs(start_twist[0]) + abs(start_twist[2] * pp.rhohat)
    pp.speedPlan(vhat_start, cruise_v, target_twist[0], u=u)

    return pp

def plan_arrays(pp, dt,
    start_pose=(0.0, 0.0, 0.0),
    start_twist=(0.0, 0.0, 0.0),
    lr_model=default_lr_model(),
    mmax=1.0,
    lr_start = (0.0, 0.0),
    details=False,
    details_v=False
):
 
    total_time = 0.0
    for seg in pp.s_plan:
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
        for segment in pp.path:
            print(fstr(segment, fmat='7.4f'))
        print('speedPlan')
        for segment in pp.s_plan:
            print(fstr(segment, fmat='7.4f'))
    num_segments = len(pp.plan)
    return (tees, lefts, rights, num_segments, total_time, vxres, omegas, poses)

if __name__ == '__main__':

    dt = 0.02
    start_pose = [0.0, 0.0, 0.0]
    start_twist = [0.0, 0.0, 0.0]
    target_pose = [.3, -.05, -D_TO_R * 180]
    target_twist = [0.0, 0.0, 0.0]
    approach_rho = 0.30
    min_rho = 0.10
    cruise_v = 0.25
    lr_start = (0.0, 0.0)
    mmax = 1.0
    u_time = 0.50
    replan_rate = 10000
    lr_model = default_lr_model()
    max_err = 0.10

    ds_lr_model = default_lr_model()

    '''
    ds_lr_model[0][1] *= .8
    ds_lr_model[0][0] *= 1.0
    ds_lr_model[2][1] *= 1.2
    ds_lr_model[2][0] *= 1.0
    '''

    dynamicStep = DynamicStep(ds_lr_model)

    ssControl = SsControl()

    zero3 = (0.0, 0.0, 0.0)
 
    pathPlot = PathPlot(history_rate=1000)
    pp = static_plan(start_pose=zero3
        , start_twist=zero3
        , target_pose=target_pose
        , target_twist=target_twist
        , approach_rho=approach_rho
        , min_rho=min_rho
        , cruise_v=cruise_v
        , u=u_time
        , details=True
    )
    (tees, lefts, rights, max_segments, plan_time, vxres, omegas, poses) = plan_arrays(pp, dt
        , start_pose=zero3
        , start_twist=zero3
        , mmax=mmax
        , details=True
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
    # the base frame (zero_map at start of plan) is the robot location. But the path
    # plan is in plan coordinates, with 0 being the wheel start.
    pose_m = zero3
    frame_m = zero3
    wheel_m = transform2d(pp.wheel_r, pose_m, zero3)
    source.noww_pose_map = wheel_m
    source.nowr_pose_map = pose_m
    for pose in poses:
        wheel_p = pose
        wheel_m = transform2d(wheel_p, pp.frame_p, frame_m)
        robot_m = transform2d(pp.robot_w, wheel_m, frame_m)
        source.pxwj.append(wheel_m[0])
        source.pywj.append(wheel_m[1])
        source.thetaj.append(wheel_m[2])
        source.pxrj.append(robot_m[0])
        source.pyrj.append(robot_m[1])
    pathPlot(dt, source)

    # now use the dynamic/control model
    twist_m = zero3
    twist_r = zero3
    next_left = lefts[0]
    next_right = rights[0]

    stepCount = 0

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

        (lr, ctl_state) = ssControl(dt, pose_m, twist_r, pp)  
      
        (pose_m, twist_m, twist_r) = dynamicStep(lr, dt, pose_m, twist_m)
        lrc = ctl_state['lr_corr']
        lrb = ctl_state['lr_base']
        print(fstr({'tt': tt, 'L0': lrb[0], 'R0': lrb[1], 'corr_left': lrc[0], 'corr_right': lrc[1], 'ctl_left': lr[0], 'ctl_right': lr[1], 'sdot': sdot, 'odot': odot}))


        if ctl_state['rms_err'] > max_err:
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

        wheel_m = transform2d(pp.wheel_r, pose_m, frame_m)

        if stepCount % 10 == 1:
            source.nowr_pose_map = pose_m
            source.noww_pose_map = wheel_m
            source.tee = tt
            source.left = lr[0]
            source.right = lr[1]
            pathPlot(dt, source)

        print(fstr({'pose_m':pose_m, 'wheel_m': wheel_m, 'twist_m': twist_m, 'twist_r': twist_r}, fmat='8.5f'))
        if ctl_state['vv']['fraction'] > 0.999:
            break

    print('all done')
    plt.waitforbuttonpress()
