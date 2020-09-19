# demo of using tensorflow gradients to set motor speeds to achieve a pose
# This uses a simple dynamic model of motion.

import os
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

import tensorflow as tf
import tensorflow.keras as keras
import math
import time

from geometry_msgs.msg import PoseStamped, Vector3, Quaternion, Transform
from bdbd_common.utils import fstr
from bdbd_common.geometry import nearPath, D_TO_R, b_to_w, lrEstimate

def dynamic_motion(lrs, lr_model, start_pose, start_twist):
    # model is in what we will call here 'base' frame
    (pxl, pxr, fx) = lr_model[0]
    (pyl, pyr, fy) = lr_model[1]
    (pol, por, fo) = lr_model[2]
    (xb, yb, thetab) = start_pose
    (vxb, vyb, omegab) = start_twist

    # robot frame velocities
    vxr = math.cos(thetab) * vxb + math.sin(thetab) * vyb
    vyr = -math.sin(thetab) * vxb + math.cos(thetab) * vyb

    t = lrs[0]['t']
    path = [{'t': t, 'pose': (xb, yb, thetab), 'twist': (vxb, vyb, omegab)}]
    for i in range(1, len(lrs)):
        # calculate motion of point in the base frame using dynamic model
        t = lrs[i]['t']
        dt = t - lrs[i-1]['t']
        left = lrs[i-1]['left']
        right = lrs[i-1]['right']

        # aply the dynamic model
        omegab += dt * (left * pol + right * por - fo * omegab)
        vxr += dt * (left * pxl + right * pxr - fx * vxr)
        vyr += dt * (left * pyl + right * pyr - fy * vyr)
        thetab += dt * omegab
        vxb = math.cos(thetab) * vxr - math.sin(thetab) * vyr
        vyb = math.sin(thetab) * vxr + math.cos(thetab) * vyr
        xb += dt * vxb
        yb += dt * vyb
        path.append([{'t': t, 'pose': (xb, yb, thetab), 'twist': (vxb, vyb, omegab)}])
        print(fstr({'vxr': vxr, 'vyr': vyr, 'vxb': vxb, 'vyb': vyb, 'omegab': omegab, 'xb': xb, 'yb': yb, 'thetab': thetab}))
    return path

# vx model
pxl = 1.258
pxr = 1.378
fx = 7.929

# vy model
pyl = -.677
pyr = .657
fy = 5.650

# omega model
pol = -7.659 # note this is negative of previous model, for consistency.
por = 7.624
fo = 8.464

lr_model = [[pxl, pxr, fx], [pyl, pyr, fy], [pol, por, fo]]

tr = 3./(fx + fy + fo) # characteristic response time of system

target_pose = [.2, .0, .3] # x, y, theta
target_twist = [0.0, 0.0, 0.0] # vx, vy, omega

start_pose = [0.0, 0.0, 0.0]
start_twist = [.30, 0.0, 0.0]

# effective x location of wheel centers relative to model
wheeldx = -((pyr - pyl) / fy) / ((por - pol) / fo)

print(' wheeldx: {:6.3f}'.format(wheeldx))

# transform model start, target values into wheel start, target values
wheel_start_pose = b_to_w(start_pose, wheeldx)
wheel_target_pose = b_to_w(target_pose, wheeldx)

print(fstr({'wheel_start_pose': wheel_start_pose, 'wheel_target_pose': wheel_target_pose}))

# determine optimal desired motion in wheel coordinates, where we can assume that frame vy = 0.
path = nearPath(wheel_target_pose[2] - wheel_start_pose[2], wheel_target_pose[0] - wheel_start_pose[0], wheel_target_pose[1] - wheel_start_pose[1])
print(fstr(path))

# estimate left, right to achieve the path
lrs = lrEstimate(path, lr_model, start_twist)

# apply the dynamic model
dynamic_path = dynamic_motion(lrs, lr_model, start_pose, start_twist)
