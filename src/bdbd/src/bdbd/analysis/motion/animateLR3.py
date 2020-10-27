# demo of using tensorflow gradients to set motor speeds to achieve a pose
# Tweak an estimate of left, right to achieve a final target

# Variation 3: RKJ notebook 2020-10-27 p 53 uses only left, right for dynamics

import os
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

import tensorflow as tf
import tensorflow.keras as keras
import math
import matplotlib.pyplot as plt
import numpy as np

from bdbd_common.utils import fstr
from bdbd_common.geometry import lr_est, default_lr_model, D_TO_R
from bdbd_common.pathPlan2 import PathPlan

class GetLoss():
    def __init__(self, start_pose, target_pose, dt, n,
        start_twist=(0.0, 0.0, 0.0),
        target_twist=(0.0, 0.0, 0.0),
        start_lr = (0.0, 0.0),
        lr_model=default_lr_model(),
        mmax=1.0,
        jerkweight=.002,
        maxweight=.0002
    ):

        self.start_pose=start_pose
        self.target_pose = target_pose
        self.dt = dt
        self.start_twist = start_twist
        self.target_twist = target_twist
        self.start_lr = start_lr
        self.lr_model = lr_model
        self.mmax = mmax
        self.jerkweight = jerkweight
        self.maxweight = maxweight

        (pxl, pxr, fx) = lr_model[0]
        (pyl, pyr, fy) = lr_model[1]
        (pol, por, fo) = lr_model[2]
        phatss = ((dt * pxl, dt * pxr), (dt * pyl, dt * pyr), (dt * pol, dt * por)) # pphats[x|y|o] [l|r]
        pphatsss = ( ([], []), ([], []), ([], []) )
        print(pphatsss[0][0])
        print(phatss[0][0])
        for j in range(3):
            for k in range(2):
                pphatsss[j][k].append(phatss[j][k])

        alphas = (1.0 - dt * fx, 1.0 - dt * fy, 1.0 - dt * fo)
        alpha_prods = [1.0, 1.0, 1.0]
        alpha_prodss = ( [alpha_prods[0]], [alpha_prods[1]], [alpha_prods[2]])

        # alpha_prodss[x|y|o][n] = alpha_prods[x|y|o]**n
        for i in range(1, n):
            for j in range(3):
                alpha_prods[j] *= alphas[j]
                alpha_prodss[j].append(alpha_prods[j])
        for alpha_prods in alpha_prodss:
            print(fstr(alpha_prods))

        # p_[x | y | o][left | right] * alpha[x | y | o] ** (-i)
        for i in range(1, n):
            for j in range(3):
                for k in range(2):
                    pphatsss[j][k].append(phatss[j][k] / alpha_prodss[j][i])

        self.tphat_xl = tf.constant(pphatsss[0][0])
        self.tphat_xr = tf.constant(pphatsss[0][1])
        self.tphat_yl = tf.constant(pphatsss[1][0])
        self.tphat_yr = tf.constant(pphatsss[1][1])
        self.tphat_ol = tf.constant(pphatsss[2][0])
        self.tphat_or = tf.constant(pphatsss[2][1])
        self.talpha_prodxs = tf.constant(alpha_prodss[0])
        self.talpha_prodys = tf.constant(alpha_prodss[1])
        self.talpha_prodos = tf.constant(alpha_prodss[2])
    
    def __call__(self, tleft, tright):

        # scalar constants
        (xb0, yb0, thetab0) = self.start_pose
        (vxb0, vyb0, omega0) = self.start_twist
        vxr0 = vxb0 * math.cos(thetab0) + vyb0 * math.cos(thetab0)
        vyr0 = -vxb0 * math.sin(thetab0) + vyb0 * math.cos(thetab0)
        dt = self.dt

        # twist vectors from left, right
        lrx_sum = tf.math.cumsum((self.tphat_xl * tleft + self.tphat_xr * tright), axis=0)
        lry_sum = tf.math.cumsum((self.tphat_yl * tleft + self.tphat_yr * tright), axis=0)
        lro_sum = tf.math.cumsum((self.tphat_ol * tleft + self.tphat_or * tright), axis=0)

        #print('tleft: ' + str(tleft))
        #print('lrx_sum: ' + str(lrx_sum))
        #print('talpha_prodxs: ' + str(self.talpha_prodxs))
        vxr = (vxr0 + lrx_sum) * self.talpha_prodxs
        vyr = (vyr0 + lry_sum) * self.talpha_prodys
        omega = (omega0 + lro_sum) * self.talpha_prodos
        vxr = tf.concat([[vxr0], vxr], axis=0, name='vxr')
        #print('vxr: ' + str(vxr))
        vyr = tf.concat([[vyr0], vyr], axis=0, name='vyr')
        omega = tf.concat([[omega0], omega], axis=0, name='omega')

        # compute new pose vectors and related vectors needed in loss

        thetab = thetab0 + dt * tf.math.cumsum(0.5 * (omega[1:] + omega[:-1]), axis=0)
        thetab = tf.concat([[thetab0], thetab], axis=0, name='thetab')
        tcos = tf.math.cos(thetab)
        tsin = tf.math.sin(thetab)
        #print(left.numpy())
        #print(right.numpy())
        vxb = tcos * vxr - tsin * vyr
        vyb = tsin * vxr + tcos * vyr
        #print('vxr after: ' + str(vxr.numpy()))
        #print('vxb: ' + str(vxb.numpy()))

        xb = xb0 + dt * tf.math.cumsum(0.5 * (vxb[1:] + vxb[:-1]))
        xb = tf.concat([[xb0], xb], axis=0, name='xb')
        #print(xb.numpy())
        yb = yb0 + dt * tf.math.cumsum(0.5 * (vyb[1:] + vyb[:-1]))
        yb = tf.concat([[yb0], yb], axis=0, name='yb')
        mexcess = (tleft / self.mmax)**10 + (tright / self.mmax)**10
        jerk = (tleft[1:] - tleft[:-1])**2 + (tright[1:] - tright[:-1])**2
        #print('jerk: ' + str(jerk.numpy()))

        # calculate the loss from the vectors
        jerk0 = (tleft[0] - self.start_lr[0])**2 + (tright[0] - self.start_lr[1])**2
        jerke = tf.concat([[jerk0], jerk], axis=0)
        jerksum = self.jerkweight * (tf.math.reduce_mean(jerke)) / dt
        maxsum = self.maxweight * tf.math.reduce_mean(mexcess)
        loss = (
            (xb[-1] - target_pose[0])**2
            + (yb[-1] - target_pose[1])**2
            + (thetab[-1] - target_pose[2])**2
            + (vxb[-1] - target_twist[0])**2
            + (vyb[-1] - target_twist[1])**2
            + (omega[-1] - target_twist[2])**2
            + tleft[-1]**2 + tright[-1]**2
            + maxsum
            + jerksum 
        )
        return (loss, maxsum, jerksum, tleft, tright, xb, yb, thetab, vxr, vyr, vxb, vyb, omega)

class TfPath():
    def __init__(self,
        mmax=1.0,
        jerkweight=.002,
        maxweight=.0002
    ):
        self.mmax = mmax
        self.jerkweight = jerkweight
        self.maxweight = maxweight

    def setup(self, start_pose, start_twist, target_pose, target_twist, cruise_v, dt,
        lr_start=(0.0, 0.0),
        lr_end=(0.0, 0.0),
    ):
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
        self.speedPlan = pp.speedPlan(start_twist[0], cruise_v, target_twist[0], u=0.50)
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
        self.getLoss = GetLoss(start_pose, target_pose, dt, len(lefts)-1,
            jerkweight=self.jerkweight,
            maxweight=self.maxweight
        )
        self.tleft = tf.Variable(lefts[1:], trainable=True, name='left_var')
        self.tright = tf.Variable(rights[1:], trainable=True, name='right_var')

        self.opt2 = keras.optimizers.Adam(learning_rate=0.05)
        self.opt1 = keras.optimizers.SGD(learning_rate=0.2)

    def tf_step(self):
        self.epoch += 1

        with tf.GradientTape(persistent=False) as tape:
            lossa = self.getLoss(self.tleft, self.tright)
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
            'omega': self.tomega[n],
            'max': max(self.tleft.numpy().max(), self.tright.numpy().max()),
            'min': min(self.tleft.numpy().min(), self.tright.numpy().min())
        }, fmat='7.5f'))
        inputs = tape.watched_variables()
        #print('inputs before: ' + str(inputs))
        gradients = tape.gradient(self.tloss, inputs)
        opt = self.opt2 if self.epoch > 5 else self.opt1
        opt.apply_gradients(zip(gradients, inputs))
        #print('inputs after: ' + str(inputs))
        #print('gradients: ' + str(gradients))

        return

start_pose = [0.0, 0.0, 0.0]
target_pose = [0.30, 0.1, 90. * D_TO_R] # x, y, theta
start_twist = [0.0, 0.0, 0.0]
target_twist = [0.0, 0.0, 0.0] # vx, vy, omega

cruise_v = 0.33
dt = 0.01
maxsteps = 400
eps = .004
jerkweight = .02
maxweight = .0002

fig = plt.figure(figsize=(8,4))
axis1 = None
axis2 = None

tfPath = TfPath(jerkweight=jerkweight, maxweight=maxweight)
tfPath.setup(start_pose, start_twist, target_pose, target_twist, cruise_v, dt)
for vv in tfPath.vvs:
    print(fstr(vv))

# tweak the dynamic model

try:
    for count in range(maxsteps):
        tfPath.tf_step()

        # graph the results
        pxs = tfPath.txb.numpy()
        pys = tfPath.tyb.numpy()
        pos = tfPath.tthetab.numpy()
        lefts = np.concatenate(([0.0], tfPath.tleft.numpy()))
        rights = np.concatenate(([0.0], tfPath.tright.numpy()))
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

        #plt1.plot(tees, txs)np.concatenate(([0.0], lefts)))
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

