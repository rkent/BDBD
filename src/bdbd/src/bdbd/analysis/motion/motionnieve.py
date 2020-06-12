'''
Nieve modeling of motion data
'''
import numpy as np
import tensorflow as tf
from tensorflow import keras
import time

OutsSeq = np.load('data/OutsSeq.npy')
InsSeq = np.load('data/InsSeq.npy')
#for i in range(1000):
#    print(InsSeq[i, :, :])

# last value in the series
batch_size = OutsSeq.shape[0]
nsteps = OutsSeq.shape[1]
nvars = OutsSeq.shape[2]

# fully-connected single layer
print(OutsSeq[0, ...].shape)
model = keras.models.Sequential([
    keras.layers.Flatten(input_shape=InsSeq[0, ...].shape),
    #keras.layers.Dense(10),
    #keras.layers.Dense(200),
    #keras.layers.Dense(100),
    #keras.layers.Dense(100),
    keras.layers.Dense(nsteps * nvars),
    keras.layers.Reshape((nsteps, nvars))
])
print(model.summary())

model.compile(loss='mean_squared_error', optimizer='Adam')
history = model.fit(InsSeq, OutsSeq, epochs=20, validation_split=0.5)

model.save('data/modelnieve')

# predictions

for i in range(300, InsSeq.shape[0]):
    pred = model.predict(InsSeq[i, np.newaxis, :, :])
    print('\n{} input:{}'.format(i, InsSeq.shape))
    for j in range(InsSeq.shape[1]):
        value = InsSeq[i, j]
        print('({:6.3f},{:6.3f}) '.format(value[0], value[1]), end=' ')

    print('\npred {}:'.format(pred.shape))
    for j in range(pred.shape[1]):
        print(' (', end='')
        for k in range(nvars):
            value = pred[0, j, k]
            print('{:6.3f}'.format(value), end= ' ')
        print(') ', end='')

    print('\noutput: {}'.format(OutsSeq.shape))
    for j in range(OutsSeq.shape[1]):
        print(' (', end='')
        for k in range(nvars):
            value = OutsSeq[i, j, k]
            print('{:6.3f}'.format(value), end= ' ')
        print(') ', end='')
    print(' ', flush=True)

    time.sleep(.10)
