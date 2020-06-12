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

# as a first attempt, I will try to predict x velocities based on inputy left, right speed

Vx = outs[:, 7, np.newaxis]
print('Vx.shape: {}'.format(Vx.shape))
print('ins.shape: {}'.format(ins.shape))
print(Vx[12000, 0])

# re-calculate Vx based on the model
#Vmax = 0.5
Vmax = 5.
f = 0.1 # contribution per time step
V = 0.0 # starting value
M = 5 # delay before response begins

for i in range(ins.shape[0]):
    if i > M:
        Vlimit = Vmax * (ins[i-M, 0] + ins[i-M, 1]) / 2.
        V = V + f * (Vlimit - V)
    Vx[i, 0] = V

# Simulate position
P = 0.0
Px = np.empty(Vx.shape)
for i in range(ins.shape[0]):
    Px[i, 0] = P + f * Vx[i, 0]

# create time series

nsteps = 50
batch_size = len(Px) - 2* nsteps

InsSeq = np.empty((batch_size, nsteps * 2, 2))
OutsSeq = np.empty((batch_size, nsteps, 1))

for i in range(batch_size):
    for j in range(nsteps * 2):
        InsSeq[i, j, :] = ins[i + j, :]
        if j >= nsteps:
            OutsSeq[i, int(j - nsteps), 0] = Px[i + j, 0] - Px[i + nsteps, 0]

print(OutsSeq.shape)
print(InsSeq.shape)
np.save('data/OutsSeq.npy', OutsSeq)
np.save('data/InsSeq.npy', InsSeq)
