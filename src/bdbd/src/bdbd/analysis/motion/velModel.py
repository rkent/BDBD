'''
Fit of motion data to an velocity model
'''
import numpy as np
import tensorflow as tf
from tensorflow import keras
import time

OutsSeq = np.load('data/OutsSeqRnnFortieth.npy')
InsSeq = np.load('data/InsSeqRnnFortieth.npy')
print(InsSeq.shape)
print(OutsSeq.shape)

#for i in range(1000):
#    print(InsSeq[i, :, :])

# last value in the series
batch_size = OutsSeq.shape[0]
nsteps = OutsSeq.shape[1]
nvars = OutsSeq.shape[2]


for k in [100, 200, 300, 400, 500]:
    print('Batch {}'.format(k))
    for j in range(nsteps):
        print('(l: {:5.2f} r: {:5.2f}) :'.format(InsSeq[k, j, 0], InsSeq[k, j, 1]), end='')
        print('{:6.3f} {:6.3f} {:6.3f}'.format(*[OutsSeq[k, j, l] for l in range(0,3)]))

'''
model = keras.models.Sequential([
    keras.layers.SimpleRNN(15, return_sequences=True, input_shape=(None, 2)),
    #keras.layers.Dense(10),
    #keras.layers.Dense(200),
    #keras.layers.Dense(100),
    #keras.layers.Dense(100),
    keras.layers.Dense(3),
])
print(model.summary())
#input('?')

model.compile(loss='mean_squared_error', optimizer='Adam')
history = model.fit(InsSeq, OutsSeq, epochs=20, validation_split=0.1)

model.save('data/modelRnnFortieth15a')
exit()
'''