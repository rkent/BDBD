# development of a path planning class.

import matplotlib.pyplot as plt
from math import cos, sin
from bdbd_common.utils import fstr
from bdbd_common.pathPlan2 import PathPlan
from bdbd_common.geometry import D_TO_R, Motor, pose2to3, transform2d, DynamicStep, default_lr_model

### MAIN PROGRAM ###

pp = PathPlan(approach_rho=0.20, min_rho=0.05, rhohat=0.2)
start_x = 0.0
start_y = 0.0
start_theta_degrees = 0.0
start_theta = start_theta_degrees * D_TO_R
end_theta_degrees = start_theta_degrees + 0.0
end_theta = end_theta_degrees * D_TO_R
end_x = start_x + .20
end_y = start_y + .1
start_omega = 0.0
start_vx_r = 0.0
start_vy_r = pp.dwheel * start_omega
end_vx_r = 0.0
over_t = 1.0
vhatcruise = 0.30

dt = 0.02

# transform base to plan coordinates
pose1 = pose2to3((start_x, start_y, start_theta))
pose2 = pose2to3((end_x, end_y, end_theta))

path_plan = pp.start(pose1, pose2)
print('path plan:')
for p in path_plan:
    print(fstr(p))

vhat0 = start_vx_r
vhatn = end_vx_r
s_plan = pp.speedPlan(vhat0, vhatcruise, vhatn, u=0.50)
print('speed plan:')
for s in s_plan:
    print(fstr(s))

tt = 0.0

motor = Motor()
speeds = []
while True:
    ss = pp.v(tt)
    ss['lr'] = motor(ss['v'], ss['omega'])
    speeds.append(ss)
    if tt > ss['time'] + over_t:
        break # at end of plan
    tt += dt

# apply the dynamic model to test accuracy
# add an error in dwheel to the dynamics

factor = 1.0
lr_model = list(default_lr_model())
lr_model[1] = list(lr_model[1])
lr_model[1][0] *= factor
lr_model[1][1] *= factor

dynamicStep = DynamicStep(lr_model)


# test of control strategy
tt = 0.0
evold = 0.0
sinpold = 0.0

# plot the movement of robot base

pose_m = pp.start_m[:]
start_vx_m = start_vx_r * cos(start_theta) - start_vy_r * sin(start_theta)
start_vy_m = start_vx_r * sin(start_theta) + start_vy_r * cos(start_theta)
twist_m = (start_vx_m, start_vy_m, start_omega)
(left, right) = speeds[1]['lr']
(leftOld, rightOld) = motor(start_vx_r, start_omega)

ii = 0
tees = []
vas = []
oas = []
psis = []
evs = []
dys = []
xr_ms = []
yr_ms = []
xw_ms = []
yw_ms = []
xpr_ms = []
ypr_ms = []
ls = []
rs = []
xn_ms = []
yn_ms = []
xp_ms = []
yp_ms = []
vns = []
ons = []
for ii in range(len(speeds)):
    tt += dt

    # dynamic model
    (pose_m, twist_m, twist_r) = dynamicStep(( 0.5 * (left + leftOld), 0.5 * (right + rightOld)), dt, pose_m, twist_m)
    leftOld = left
    rightOld = right

    # control step
    (v_new, o_new) = pp.controlStep(dt, pose_m, twist_r)

    (left, right) = motor(v_new, o_new)
    '''
    # introduce an error
    left *= random.gauss(0.95, 0.05)
    right *= random.gauss(0.9, 0.05)
    '''
    #(left, right) = speeds[ii]['lr']
    # speeds[]['point'] is the progression of the wheels starting at (0.0, 0.0, 0.0), ie in frame_p
    # get wheels_m
    plan_w_p = speeds[ii]['point']
    plan_w_m = transform2d(plan_w_p, pp.frame_p, pp.frame_m)
    # plan_w_m is also the w frame in the plan. Get robot position in that frame
    plan_r_m = transform2d(pp.robot_w, plan_w_m, pp.frame_m)

    real_wheel_m = transform2d(pp.wheel_r, pose_m, pp.frame_m)

    print(' ')
    print(fstr({
        't': tt,
        'lr': (left, right),
        'pose_m': pose_m,
        'v_old': pp.va,
        'v_new': v_new,
        'vhat_old': pp.vhata,
        'vhat_new': pp.vhat_new,
        'o_old': pp.oa,
        'o_new': o_new,
        'kappaa': pp.kappaa,
        'kappa_new': pp.kappa_new,
    }))
    '''
    print(fstr({
        'nearest_w': nearest_w,
        'nearest_m': nearest_m,
        'dy_w': dy_w,
        'kappa_near': kappa_near,
        'sin(psi)': sin(psi)
        }))

    print(fstr({
        'wheel_pose_p': wheel_pose_p,
        'wheel_r': pp.wheel_r,
        'pose_m': pose_m,
        'nearest_m': nearest_m,
        'nearest_p': nearest_p,
    }))

    print(fstr({
        't': tt,
        'lr': (left, right),
        'dyn_w_m': pp.wheel_pose_m,
        'plan_w_m': plan_w_m,
        'dyn_r_m': pose_m,
        'plan_r_m': plan_r_m,
        #'model_t': twist_r, 
        }))
    '''


    print(fstr({
        'y_error': pp.dy_r,
        'psi_e_deg': pp.psi / D_TO_R,
        'sin(psi)': sin(pp.psi),
        'dydt': pp.dydt,
        'dsinpt': pp.dsinpdt,
        'near_wheel_p': pp.near_wheel_p,
    }))
    #print(fstr({'best_fraction': best_fraction, 'best_segment': best_segment}))
    near_wheel_m = transform2d(pp.near_wheel_p, pp.frame_p, pp.frame_m)
    tees.append(tt)
    vas.append(pp.va)
    oas.append(pp.oa)
    psis.append(sin(pp.psi))
    evs.append(pp.ev)
    dys.append(100.0 * pp.dy_r)
    xr_ms.append(pose_m[0])
    yr_ms.append(pose_m[1])
    xw_ms.append(real_wheel_m[0])
    yw_ms.append(real_wheel_m[1])
    ls.append(left)
    rs.append(right)
    xp_ms.append(plan_w_m[0])
    yp_ms.append(plan_w_m[1])
    xpr_ms.append(plan_r_m[0])
    ypr_ms.append(plan_r_m[1])
    vns.append(v_new)
    ons.append(o_new)

fig = plt.figure(figsize=(6,6))

'''
plt.plot(tees, vas)
plt.plot(tees, oas)
plt.plot(tees, vns)
plt.plot(tees, ons)
plt.plot(tees, psis)
#plt.plot(tees, evs)
plt.plot(tees, dys)
#plt.plot(tees, xr_ms)
#plt.plot(tees, yr_ms)
#plt.plot(tees, xw_ms)
#plt.plot(tees, yw_ms)
plt.plot(tees, ls)
plt.plot(tees, rs)
'''
plt.axis('equal')
plt.plot(xpr_ms, ypr_ms)
plt.plot(xp_ms, yp_ms)
plt.plot(xr_ms, yr_ms)
plt.plot(xw_ms, yw_ms)
#plt.plot(xn_ms, yn_ms)
'''
'''
plt.waitforbuttonpress()
