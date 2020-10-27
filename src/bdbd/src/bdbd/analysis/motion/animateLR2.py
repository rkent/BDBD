# demo of using tensorflow gradients to set motor speeds to achieve a pose
# Tweak an estimate of left, right to achieve a final target

# Variation 2: try to use a tensorflow functions

import os
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

import tensorflow as tf
import tensorflow.keras as keras
import math
import matplotlib.pyplot as plt

from bdbd_common.utils import fstr
from bdbd_common.geometry import lr_est
from bdbd_common.pathPlan2 import PathPlan

#@tf.function
def get_loss(left, right, vxr, vyr, omega, start_pose, start_twist, target_pose, target_twist, lr_model, dt, mmax, jerkweight, maxweight):

    # debug print of 4th line
    #print(left[4].numpy(), right[4].numpy(), vxr[4].numpy(), vyr[4].numpy(), omega[4].numpy())
    #print(dt, start_pose, target_pose, start_twist, target_twist)
    #print(lr_model[0])

    # scalar constants
    (pxl, pxr, fx) = lr_model[0]
    (pyl, pyr, fy) = lr_model[1]
    (pol, por, fo) = lr_model[2]
    (xb0, yb0, thetab0) = start_pose
    (vxb0, vyb0, omega0) = start_twist
    vxr0 = vxb0 * math.cos(thetab0) + vyb0 * math.cos(thetab0)
    vyr0 = -vxb0 * math.sin(thetab0) + vyb0 * math.cos(thetab0)

    # compute new pose and twist vectors and related vectors needed in loss
    #omega_new = omega0 + dt * (pol * left[1:] + por * right[1:] - 0.5 * fo * (omega[1:] + omega[:-1]))
    omega_new = omega0 + dt * (pol * left[1:] + por * right[1:] - fo * omega[:-1])
    omega = tf.concat([[omega[0]], omega_new], axis=0)
    thetab_new = thetab0 + dt * tf.math.cumsum(0.5 * (omega[1:] + omega[:-1]))
    thetab = tf.concat([[thetab0], thetab_new], axis=0, name='thetab')
    tcos = tf.math.cos(thetab)
    tsin = tf.math.sin(thetab)
    #print(left.numpy())
    #print(right.numpy())
    #print('vxr before: ' + str(vxr.numpy()))
    #print(fx * vxr)
    #print(pxl * left + pxr * right - fx * vxr)
    vxr_new = vxr0 + dt * tf.math.cumsum(pxl * left[1:] + pxr * right[1:] - fx * vxr[:-1])
    vxr = tf.concat([[vxr0], vxr_new], axis=0, name='vxr')

    vyr_new = vyr0 + dt * tf.math.cumsum(pyl * left[1:] + pyr * right[1:] - fy * vyr[:-1])
    vyr = tf.concat([[vyr0], vyr_new], axis=0, name='vyr')
    vxb = tcos * vxr - tsin * vyr
    vyb = tsin * vxr + tcos * vyr
    #print('vxr after: ' + str(vxr.numpy()))
    #print('vxb: ' + str(vxb.numpy()))

    #xb_new = xb0 + dt * tf.math.cumsum(0.5 * (vxr[1:] + vxr[:-1]))
    xb_new = xb0 + dt * tf.math.cumsum(vxr[:-1])
    xb = tf.concat([[xb0], xb_new], axis=0, name='xb')
    #print(xb.numpy())
    #yb_new = yb0 + dt * tf.math.cumsum(0.5 * (vyr[1:] + vyr[:-1]))
    yb_new = yb0 + dt * tf.math.cumsum(vyr[:-1])
    yb = tf.concat([[yb0], yb_new], axis=0, name='yb')
    mexcess = (left / mmax)**8 + (right / mmax)**8
    jerk = (left[1:] - left[:-1])**2 + (right[1:] - right[:-1])**2
    #print('jerk: ' + str(jerk.numpy()))

    # calculate the loss from the vectors
    jerksum = jerkweight * tf.math.reduce_sum(jerk) / dt
    maxsum = maxweight * tf.math.reduce_sum(mexcess)
    loss = (
        (xb[-1] - target_pose[0])**2
        #+ (yb[-1] - target_pose[1])**2
        #+ (thetab[-1] - target_pose[2])**2
        + (vxb[-1] - target_twist[0])**2
        + 100. * (vyb[-1] - target_twist[1])**2
        + (omega[-1] - target_twist[2])**2
        #+ left[-1]**2 + right[-1]**2
        # + maxsum
        #+ jerksum 
    )
    return (loss, maxsum, jerksum, left, right, xb, yb, thetab, vxr, vyr, vxb, vyb, omega)

class TfPath():
    def __init__(self,
        mmax=1.0,
        jerkweight=1.e-8,
        maxweight=1.e-8
    ):
        self.mmax = mmax
        self.jerkweight = jerkweight
        self.maxweight = maxweight

    def setup(self, start_pose, start_twist, target_pose, target_twist, cruise_v, dt, lr_start=(0.0, 0.0), lr_end=(0.0, 0.0)):
        self.start_pose = start_pose
        self.start_twist = start_twist
        self.target_pose = target_pose
        self.target_twist = target_twist
        self.lr_start = lr_start
        self.lr_end = lr_end
        self.epoch = 0
        self.dt = dt

        # model is in what we will call here 'base' frame
        pp = PathPlan()
        self.pp = pp

        print(fstr({'target_pose': target_pose, 'target_twist': target_twist}))
        self.pathPlan = pp.start2(start_pose, target_pose)
        print('path_plan:')
        for segment in self.pathPlan:
            print(fstr(segment))

        # estimate left, right to achieve the path
        self.speedPlan = pp.speedPlan(start_twist[0], cruise_v, target_twist[0])
        print('speed_plan:')
        for segment in self.speedPlan:
            print(fstr(segment))

        print('estimate lr without dynamics')
        vxr0 = start_twist[0] * math.cos(start_pose[2]) + start_twist[1] * math.sin(start_pose[2])
        vyr0 = -start_twist[0] * math.sin(start_pose[2]) + start_twist[1] * math.cos(start_pose[2])
        last_vx = vxr0
        last_omega = start_twist[2]

        lefts = [lr_start[0]]
        rights = [lr_start[1]]
        vxres = [vxr0]
        vyres = [vyr0]
        omegas = [self.start_twist[2]]
        self.vvs = [self.pp.v(0.0)]

        tt = 0.0
        while True:
            tt += dt
            vv = self.pp.v(tt)
            self.vvs.append(vv)
            # vv gives vhat is in wheel frame. We need to convert to robot frame.
            vxres.append(vv['v'])
            vyres.append(vv['omega'] * pp.dwheel)
            omegas.append(vv['omega'])

            (left, right, last_vx, last_omega) = lr_est(vv['v'], vv['omega'], last_vx, last_omega, dt)
            lefts.append(left)
            rights.append(right)

            if vv['fraction'] > 0.9999:
                break

        '''
        self.tleft = tf.concat(
            (   tf.constant([lefts[0]]),
                tf.Variable(lefts[1:], trainable=True, name='left_var')
            ), axis=0, name='tleft')
        self.tright = tf.concat(
            (   tf.constant([rights[0]]),
                tf.Variable(rights[1:], trainable=True, name='right_var')
            ), axis=0, name='tright')
        '''
        self.tleft = tf.Variable(lefts, trainable=True, name='left_var')
        self.tright = tf.Variable(rights, trainable=True, name='right_var')
        self.tvxr = tf.constant(vxres, name='vxr')
        self.tvyr = tf.constant(vyres, name='vyr')
        self.tomega = tf.constant(omegas, name='omega')

        self.opt2 = keras.optimizers.Adam(learning_rate=.000000002)
        self.opt1 = keras.optimizers.SGD(learning_rate=0.000002)

    def tf_step(self):
        self.epoch += 1

        with tf.GradientTape(persistent=False) as tape:
            lossa = get_loss(
                self.tleft,
                self.tright,
                self.tvxr,
                self.tvyr,
                self.tomega,
                self.start_pose,
                self.start_twist,
                self.target_pose,
                self.target_twist,
                self.pp.lr_model,
                self.dt,
                self.mmax,
                self.jerkweight,
                self.maxweight
            )
            (
                self.tloss,
                self.tmaxsum,
                self.tjerksum,
                self.tleft,
                self.tright,
                self.txb,
                self.tyb,
                self.tthetab,
                self.tvxr,
                self.tvyr,
                self.tvxb,
                self.tvyb,
                self.tomega
            ) = lossa

        n = self.tleft.shape[0] - 1
        print(fstr({
            'i': self.epoch,
            'loss': self.tloss,
            'jerk': self.tjerksum,
            'maxs': self.tmaxsum,
            'x': self.txb[n],
            'y': self.tyb[n],
            'theta': self.tthetab[n],
            'vx': self.tvxb[n],
            'vy': self.tvyb[n],
            'omega': self.tomega[n]
        }, fmat='7.5f'))
        inputs = tape.watched_variables()
        #print('inputs before: ' + str(inputs))
        gradients = tape.gradient(self.tloss, inputs)
        opt = self.opt2 if self.epoch > 100 else self.opt1
        opt.apply_gradients(zip(gradients, inputs))
        #print('inputs after: ' + str(inputs))
        #print('gradients: ' + str(gradients))

        return

start_pose = [0.0, 0.0, 0.0]
target_pose = [0.15, 0.0, 0.0] # x, y, theta
start_twist = [0.0, 0.0, 0.0]
target_twist = [0.0, 0.0, 0.0] # vx, vy, omega

cruise_v = 0.25
dt = 0.02
maxsteps = 100
eps = .004

fig = plt.figure(figsize=(8,4))
axis1 = None
axis2 = None

tfPath = TfPath()
tfPath.setup(start_pose, start_twist, target_pose, target_twist, cruise_v, dt)
for vv in tfPath.vvs:
    print(fstr(vv))

# tweak the dynamic model

path = None
try:
    for count in range(maxsteps):
        tfPath.tf_step()

        # graph the results
        pxs = tfPath.txb.numpy()
        pys = tfPath.tyb.numpy()
        pos = tfPath.tthetab.numpy()
        lefts = tfPath.tleft.numpy()
        rights = tfPath.tright.numpy()
        txs = tfPath.tvxb.numpy()
        tys = tfPath.tvyb.numpy()
        tos = tfPath.tomega.numpy()

        tees = []
        for p in tfPath.vvs:
            tees.append(p['time'])

        fig.clf()
        plt1 = fig.add_subplot(121)
        #plt1.axis([0.0, tfPath.lrs[-1]['t'], -1.5, 1.5])
        plt2 = fig.add_subplot(122)

        if axis1 is not None:
            plt1.axis(axis1)
        if axis2 is not None:
            plt2.axis(axis2)
        else:
            plt2.axis('equal')

        #plt2.axis([-0.00, 0.40, -0.05, 0.40])

        #plt1.plot(tees, txs)
        plt1.plot(tees, tys)
        plt1.plot(tees, lefts)
        plt1.plot(tees, rights)
        plt1.plot(tees, tos)

        plt2.plot(pxs, pys)

        plt.pause(0.001)
        if axis1 is None:
            axis1 = plt1.axis()
        if axis2 is None:
            axis2 = plt2.axis()

        #if tfPath.tloss < eps:
        #    break

    plt.waitforbuttonpress()

except KeyboardInterrupt:
    pass

