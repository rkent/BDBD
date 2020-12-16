# demo of using tensorflow gradients to set motor speeds to achieve a pose
# Tweak an estimate of left, right to achieve a final target

import os
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

import tensorflow as tf
import tensorflow.keras as keras
import math
import matplotlib.pyplot as plt

from bdbd_common.utils import fstr
from bdbd_common.geometry import nearPath, D_TO_R, b_to_w, dynamic_motion, Motor
from bdbd_common.pathPlan2 import PathPlan

class TfPath():
    def __init__(self,
        mmax=1.0,
        jerkweight=0.2,
        maxweight=1.e-5
    ):
        self.mmax = mmax
        self.jerkweight = jerkweight
        self.maxweight = maxweight

    def setup(self, start_pose, start_twist, target_pose, target_twist, cruise_v, dt):
        self.start_pose = start_pose
        self.start_twist = start_twist
        self.target_pose = target_pose
        self.target_twist = target_twist
        (xb, yb, thetab) = start_pose
        (vxb, vyb, omegab) = start_twist
        self.epoch = 0
        self.dt = dt

        # model is in what we will call here 'base' frame
        pp = PathPlan()
        self.pp = pp

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
        tt = 0.0
        motor = Motor()
        lrs = []
        self.lrs = lrs
        self.vvs = []
        while True:
            vv = self.pp.v(tt)
            self.vvs.append(vv)
            lr = motor(vv['v'], vv['omega'])
            lrs.append({'t': tt, 'left': lr[0], 'right': lr[1]})
            print(fstr(lrs[-1]) + fstr(vv))
            if vv['fraction'] > 0.9999:
                break
            tt += dt

        # copy trainable estimates into tensors
        tleft = []
        tright = []
        for i in range(len(lrs)):
            tleft.append(tf.Variable(lrs[i]['left'], name='left'+str(i)))
            tright.append(tf.Variable(lrs[i]['right'], name='right'+str(i)))

        # accumulating variables
        tomegab = tf.TensorArray(dtype=tf.float32, size=len(lrs), dynamic_size=False, clear_after_read=False, name='omegab')
        self.tomegab = tomegab.write(0, omegab)
        tvxb = tf.TensorArray(dtype=tf.float32, size=len(lrs), dynamic_size=False, clear_after_read=False, name='vxb')
        self.tvxb = tvxb.write(0, vxb)
        tvyb = tf.TensorArray(dtype=tf.float32, size=len(lrs), dynamic_size=False, clear_after_read=False, name='vyb')
        self.tvyb = tvyb.write(0, vyb)
        tthetab = tf.TensorArray(dtype=tf.float32, size=len(lrs), dynamic_size=False, clear_after_read=False, name='theta')
        self.tthetab = tthetab.write(0, thetab)
        txb = tf.TensorArray(dtype=tf.float32, size=len(lrs), dynamic_size=False, clear_after_read=False, name='txb')
        self.txb = txb.write(0, xb)
        tyb = tf.TensorArray(dtype=tf.float32, size=len(lrs), dynamic_size=False, clear_after_read=False, name='tyb')
        self.tyb = tyb.write(0, yb)

        self.opt2 = keras.optimizers.Adam(learning_rate=.02)
        self.opt1 = keras.optimizers.SGD(learning_rate=0.2)
        self.tleft = tleft
        self.tright = tright

    def tf_step(self):
        self.epoch += 1
        (pxl, pxr, fx) = self.pp.lr_model[0]
        (pyl, pyr, fy) = self.pp.lr_model[1]
        (pol, por, fo) = self.pp.lr_model[2]
        thetab = self.start_pose[2]
        (vxb, vyb, omegab) = self.start_twist
        tleft = self.tleft
        tright = self.tright

        # robot frame velocities
        vxr = math.cos(thetab) * vxb + math.sin(thetab) * vyb
        vyr = -math.sin(thetab) * vxb + math.cos(thetab) * vyb

        with tf.GradientTape(persistent=True) as tape:
            jerksum = 0.0
            maxsum = 0.0
            for i in range(1, len(self.lrs)):
                left= 0.5*(tleft[i-1] + tleft[i])
                right = 0.5*(tright[i-1] + tright[i])
                omegabOld = self.tomegab.read(i-1)
                thetab = self.tthetab.read(i-1)
                omegab = omegabOld + dt * ((left * pol + right * por - fo * omegabOld))
                self.tomegab = self.tomegab.write(i, omegab)
                vxrOld = vxr
                vyrOld = vyr
                vxr = vxrOld + dt * (left * pxl + right * pxr - fx * vxrOld)
                vyr = vyrOld + dt * (left * pyl + right * pyr - fy * vyrOld)
    
                # these are needed for the loss calculation
                jerksum += (tleft[i] - tleft[i-1])**2 + (tright[i] - tright[i-1])**2
                maxsum += (left / self.mmax)**10 + (right / self.mmax)**8
                thetab = thetab + 0.5 * dt * (omegab + omegabOld)
                self.tthetab = self.tthetab.write(i, thetab)
                tcos = tf.math.cos(thetab)
                tsin = tf.math.sin(thetab)
                vxb = tcos * vxr - tsin * vyr
                vyb = tsin * vxr + tcos * vyr
                self.tvxb = self.tvxb.write(i, vxb)
                self.tvyb = self.tvyb.write(i, vyb)
                self.txb = self.txb.write(i, self.txb.read(i-1) + dt * 0.5 * (vxb + self.tvxb.read(i-1)))
                self.tyb = self.tyb.write(i, self.tyb.read(i-1) + dt * 0.5 * (vyb + self.tvyb.read(i-1)))

            n = len(self.lrs) - 1
            loss = (
                (self.txb.read(n) - self.target_pose[0])**2 +
                (self.tyb.read(n) - self.target_pose[1])**2 +
                (self.tthetab.read(n) - self.target_pose[2])**2 +
                (self.tvxb.read(n) - self.target_twist[0])**2 +
                (self.tvyb.read(n) - self.target_twist[1])**2 +
                (self.tomegab.read(n) - self.target_twist[2])**2 +
                tleft[n]**2 + tright[n]**2 +
                self.jerkweight * jerksum +
                self.maxweight * maxsum
            )

        print(fstr({
            'i': self.epoch,
            'loss': loss.numpy(),
            'jerk': self.jerkweight * jerksum.numpy(),
            'maxs': self.maxweight * maxsum,
            'x': self.txb.read(n).numpy(),
            'y': self.tyb.read(n).numpy(),
            'theta': self.tthetab.read(n).numpy(),
            'vx': self.tvxb.read(n),
            'vy': self.tvyb.read(n),
            'omega': self.tomegab.read(n)
        }, fmat='7.5f'))
        inputs = tape.watched_variables()
        gradients = tape.gradient(loss, inputs)
        opt = self.opt2 if self.epoch > 5 else self.opt1
        opt.apply_gradients(zip(gradients, inputs))

        path = []
        t = 0.0
        for i in range(len(self.lrs)):
            path.append({
                't': t,
                'pose': (self.txb.read(i).numpy(), self.tyb.read(i).numpy(), self.tthetab.read(i).numpy()),
                'lr': (self.tleft[i].numpy(), self.tright[i].numpy()),
                'twist': (self.tvxb.read(i).numpy(), self.tvyb.read(i).numpy(), self.tomegab.read(i).numpy())
            })
            t += dt

        self.loss = loss
        self.jerksum = jerksum
        self.maxsum = maxsum

        return path

start_pose = [0.0, 0.0, 0.0]
target_pose = [0.3, 0.1, 180. * D_TO_R] # x, y, theta
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
        path = tfPath.tf_step()

        # graph the results
        tees = []
        pxs = []
        pys = []
        pos = []
        lefts = []
        rights = []
        txs = []
        tys = []
        tos = []

        for p in path:
            tees.append(p['t'])
            pxs.append(p['pose'][0])
            pys.append(p['pose'][1])
            pos.append(p['pose'][2])
            lefts.append(p['lr'][0])
            rights.append(p['lr'][1])
            txs.append(p['twist'][0])
            tys.append(p['twist'][1])
            tos.append(p['twist'][2])

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

        plt1.plot(tees, txs)
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

        if tfPath.loss < eps:
            break

    plt.waitforbuttonpress()

except KeyboardInterrupt:
    pass

for p in path:
    print(fstr(p))

