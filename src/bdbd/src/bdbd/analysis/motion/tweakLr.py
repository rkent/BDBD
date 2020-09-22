# demo of using tensorflow gradients to set motor speeds to achieve a pose
# Tweak an estimate of left, right to achieve a final target

import os
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

import tensorflow as tf
import tensorflow.keras as keras
import math
import time

from geometry_msgs.msg import PoseStamped, Vector3, Quaternion, Transform
from bdbd_common.utils import fstr
from bdbd_common.geometry import nearPath, D_TO_R, b_to_w, lrEstimate, dynamic_motion

def path_from_lr(lrs, lr_model, start_pose, start_twist, target_pose, target_twist, mmax=1.0, error=.01):
    # model is in what we will call here 'base' frame
    (pxl, pxr, fx) = lr_model[0]
    (pyl, pyr, fy) = lr_model[1]
    (pol, por, fo) = lr_model[2]
    (xb, yb, thetab) = start_pose
    (vxb, vyb, omegab) = start_twist

    # robot frame velocities
    vxr0 = math.cos(thetab) * vxb + math.sin(thetab) * vyb
    vyr0 = -math.sin(thetab) * vxb + math.cos(thetab) * vyb

    dt = lrs[1]['t'] - lrs[0]['t']

    # copy trainable estimates into tensors
    tleft = []
    tright = []
    for i in range(len(lrs)):
        tleft.append(tf.Variable(lrs[i]['left'], name='left'+str(i)))
        tright.append(tf.Variable(lrs[i]['right'], name='right'+str(i)))

    # accumulating variables
    tomegab = tf.TensorArray(dtype=tf.float32, size=len(lrs), dynamic_size=False, clear_after_read=False, name='omegab')
    tomegab = tomegab.write(0, omegab)
    tvxb = tf.TensorArray(dtype=tf.float32, size=len(lrs), dynamic_size=False, clear_after_read=False, name='vxb')
    tvxb = tvxb.write(0, vxb)
    tvyb = tf.TensorArray(dtype=tf.float32, size=len(lrs), dynamic_size=False, clear_after_read=False, name='vyb')
    tvyb = tvyb.write(0, vyb)
    tthetab = tf.TensorArray(dtype=tf.float32, size=len(lrs), dynamic_size=False, clear_after_read=False, name='theta')
    tthetab = tthetab.write(0, thetab)
    txb = tf.TensorArray(dtype=tf.float32, size=len(lrs), dynamic_size=False, clear_after_read=False, name='txb')
    txb = txb.write(0, xb)
    tyb = tf.TensorArray(dtype=tf.float32, size=len(lrs), dynamic_size=False, clear_after_read=False, name='tyb')
    tyb = tyb.write(0, yb)

    opt2 = keras.optimizers.Adam(learning_rate=.02)
    opt1 = keras.optimizers.SGD(learning_rate=0.5)
    for epoch in range(0, 1000):
        vxr = vxr0
        vyr = vyr0
        with tf.GradientTape(persistent=True) as tape:
            for i in range(1, len(lrs)):
                left = tf.math.maximum(-mmax, tf.math.minimum(mmax, 0.5*(tleft[i-1] + tleft[i])))
                right = tf.math.maximum(-mmax, tf.math.minimum(mmax, 0.5*(tright[i-1] + tright[i])))
                omegabOld = tomegab.read(i-1)
                thetab = tthetab.read(i-1)

                omegab = omegabOld + dt * ((left * pol + right * por - fo * omegabOld))
                tomegab = tomegab.write(i, omegab)
                vxrOld = vxr
                vyrOld = vyr
                vxr = vxrOld + dt * (left * pxl + right * pxr - fx * vxrOld)
                vyr = vyrOld + dt * (left * pyl + right * pyr - fy * vyrOld)
    
                # these are needed for the loss calculation
                thetab = thetab + 0.5 * dt * (omegab + omegabOld)
                tthetab = tthetab.write(i, thetab)
                tcos = tf.math.cos(thetab)
                tsin = tf.math.sin(thetab)
                vxb = tcos * vxr - tsin * vyr
                vyb = tsin * vxr + tcos * vyr
                tvxb = tvxb.write(i, vxb)
                tvyb = tvyb.write(i, vyb)
                txb = txb.write(i, txb.read(i-1) + dt * 0.5 * (vxb + tvxb.read(i-1)))
                tyb = tyb.write(i, tyb.read(i-1) + dt * 0.5 * (vyb + tvyb.read(i-1)))

            n = len(lrs) - 1
            loss = (
                (txb.read(n) - target_pose[0])**2 +
                (tyb.read(n) - target_pose[1])**2 +
                (tthetab.read(n) - target_pose[2])**2 +
                (tvxb.read(n) - target_twist[0])**2 +
                (tvyb.read(n) - target_twist[1])**2 +
                (tomegab.read(n) - target_twist[2])**2
            )
        print(fstr({'epoch': epoch, 'loss': loss.numpy(), 'xb': txb.read(n).numpy(), 'yb': tyb.read(n).numpy(), 'theta': tthetab.read(n).numpy()}, fmat='7.5f'))
        if loss.numpy() < error:
            break
        inputs = tape.watched_variables()
        gradients = tape.gradient(loss, inputs)
        opt = opt2 if epoch > 5 else opt1
        opt.apply_gradients(zip(gradients, inputs))

    path = []
    t = 0.0
    for i in range(len(lrs)):
        path.append({
            't': t,
            'pose': (txb.read(i).numpy(), tyb.read(i).numpy(), tthetab.read(i).numpy()),
            'lr': (tleft[i].numpy(), tright[i].numpy()),
            'twist': (tvxb.read(i).numpy(), tvyb.read(i).numpy(), tomegab.read(i).numpy())
        })
        t += dt

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

target_pose = [.20, .1, .3] # x, y, theta
target_twist = [0.0, 0.0, 0.0] # vx, vy, omega

start_pose = [0.0, 0.0, 0.0]
start_twist = [.3, 0.0, 0.0]

# effective x location of wheel centers relative to model
wheeldx = -((pyr - pyl) / fy) / ((por - pol) / fo)

print(' wheeldx: {:6.3f}'.format(wheeldx))

# transform model start, target values into wheel start, target values
wheel_start_pose = b_to_w(start_pose, wheeldx)
wheel_target_pose = b_to_w(target_pose, wheeldx)

print(fstr({'wheel_start_pose': wheel_start_pose, 'wheel_target_pose': wheel_target_pose}))

# determine optimal desired motion in wheel coordinates, where we can assume that frame vy = 0.
path = nearPath(wheel_target_pose[0] - wheel_start_pose[0], wheel_target_pose[1] - wheel_start_pose[1], wheel_target_pose[2] - wheel_start_pose[2])
print(fstr(path))

# estimate left, right to achieve the path
dt = 0.05
print('estimate the path without dynamics')
lrs = lrEstimate(path, lr_model, start_twist, dt)

# apply the dynamic model
print('apply the dynamic model to test the estimate')
dynamic_path = dynamic_motion(lrs, start_pose)
for item in dynamic_path:
    print(fstr(item))

# tweak the dynamic model
start = time.time()

print('tweak l/r to account for dynamics')
path = path_from_lr(
    lrs, lr_model, start_pose, start_twist, target_pose, target_twist, error=0.0001, mmax=1.0)

for item in path:
    print(fstr(item))

print('elapsed time: {:6.3f}'.format(time.time() - start))
# recreate lrs and reapply dynamic model
lrs = []
t = 0.
for row in path:
    lrs.append({'t': row['t'], 'left': row['lr'][0], 'right': row['lr'][1]})
    t += dt

print('reapplying the dynamic model')
dynamic_path = dynamic_motion(lrs, start_pose, start_twist, lr_model)
for item in dynamic_path:
    print(fstr(item))
