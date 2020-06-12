'''
pre-process motion data into a form ready for modeling
'''
import numpy as np
import tensorflow as tf
from tensorflow import keras
import time

'''

    definition of the outputs array:

    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    l = msg.twist.twist.linear
    a = msg.twist.twist.angular
    output_sums[0] += p.x
    output_sums[1] += p.y
    output_sums[2] += p.z
    output_sums[3] += o.x
    output_sums[4] += o.y
    output_sums[5] += o.z
    output_sums[6] += o.w
    output_sums[7] += l.x
    output_sums[8] += l.y
    output_sums[9] += l.z
    output_sums[10] += a.x
    output_sums[11] += a.y
    output_sums[12] += a.z
'''
outs = np.load('data/outputs.npy')
ins = np.load('data/inputs.npy')

Vx = outs[:, 7]
Vy = outs[:, 8]
Px = outs[:, 0]
print('Vx.shape: {}'.format(Vx.shape))
print('ins.shape: {}'.format(ins.shape))

'''
# just show values
for i in range(400, len(Vx)):
    print('i: {} left: {:6.3f} right: {:6.3f} Px: {:6.3f} Vx:{:6.3f}'.format(i, ins[i,0], ins[i,1], Px[i, 0], Vx[i, 0]))
    time.sleep(.1)
'''

'''
VxMean = np.mean(Vx, axis=0, keepdims=True)
VxStd = np.std(Vx, axis=0, keepdims=True)
VxMax = np.max(Vx, axis=0, keepdims=True)
VxMin = np.min(Vx, axis=0, keepdims=True)
print(VxMean.shape)
print(VxMean, VxStd, VxMax, VxMin)

VxNormed = -1. + 2.* (Vx - VxMin) / (VxMax - VxMin + keras.backend.epsilon())
'''
# skipped normalization
VxNormed = Vx
print('VxNormed.shape: {}'.format(VxNormed.shape))

# create time series

nsteps = 50
batch_size = len(VxNormed) - 2* nsteps

InsSeq = np.empty((batch_size, nsteps * 2, 2))
OutsSeq = np.empty((batch_size, nsteps, 2))

for i in range(batch_size):
    for j in range(nsteps * 2):
        InsSeq[i, j, :] = ins[i + j, :]
        if j >= nsteps:
            OutsSeq[i, int(j - nsteps), 0] = VxNormed[i + j]
            OutsSeq[i, int(j - nsteps), 1] = Px[i + j] - Px[i + nsteps]


print(OutsSeq.shape)
print(InsSeq.shape)

# show results
for i in range(1400, batch_size):
    print('i: {} l: {:6.3f} right: {:6.3f} Px: {:6.3f} Vx:{:6.3f} Vy:{:6.3f}'.format(i, ins[i,0], ins[i,1], Px[i], Vx[i], Vy[i]))
    '''
    print('Px: (', end='')
    for j in range(nsteps):
        print(' {:6.3f}'.format(OutsSeq[i, j, 1]), end='')
    print(')')
    '''
    time.sleep(0.1)