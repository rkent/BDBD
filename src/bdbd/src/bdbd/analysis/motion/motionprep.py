'''
pre-process motion data into a form ready for modeling
'''
import numpy as np
import tensorflow as tf
from tensorflow import keras

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

# as a first attempt, I will try to predict x velocities and position based on input left, right speed

Vx = outs[:, 7]
Vy = outs[:, 8]
Az = outs[:, 12]
print('Vx.shape: {}'.format(Vx.shape))
print('ins.shape: {}'.format(ins.shape))

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

# create time series

nsteps = 50
batch_size = len(Vx) - 2* nsteps

InsSeq = np.empty((batch_size, nsteps * 2, 2))
OutsSeq = np.empty((batch_size, nsteps, 3))

for i in range(batch_size):
    for j in range(nsteps * 2):
        InsSeq[i, j, :] = ins[i + j, :]
        if j >= nsteps:
            OutsSeq[i, int(j - nsteps), 0] = Vx[i + j]
            OutsSeq[i, int(j - nsteps), 1] = Vy[i + j]
            OutsSeq[i, int(j - nsteps), 2] = Az[i + j]

print(OutsSeq.shape)
print(InsSeq.shape)
np.save('data/OutsSeq.npy', OutsSeq)
np.save('data/InsSeq.npy', InsSeq)
