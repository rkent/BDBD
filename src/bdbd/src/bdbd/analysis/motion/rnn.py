'''
Nieve modeling of motion data
'''
import numpy as np
import tensorflow as tf
from tensorflow import keras
import time

OutsSeq = np.load('data/OutsSeqRnn.npy')
InsSeq = np.load('data/InsSeqRnn.npy')
#for i in range(1000):
#    print(InsSeq[i, :, :])

# last value in the series
batch_size = OutsSeq.shape[0]
nsteps = OutsSeq.shape[1]
nvars = OutsSeq.shape[2]

print(InsSeq.shape)
print(OutsSeq.shape)
'''
for k in range(1000):
    print('InsSeq:')
    for j in range(100):
        print('({:5.2f},{:5.2f}) '.format(InsSeq[k, j, 0], InsSeq[k, j, 1]), end='')
    print('')
    print('OutsSeq:')
    for j in range(100):
        print('{:6.3f} '.format(OutsSeq[k, j, 0]), end='')
    print('')
'''
model = keras.models.Sequential([
    keras.layers.SimpleRNN(50, return_sequences=True, input_shape=(None, 2)),
    #keras.layers.Dense(10),
    #keras.layers.Dense(200),
    #keras.layers.Dense(100),
    #keras.layers.Dense(100),
    keras.layers.Dense(3),
])
print(model.summary())
input('?')

model.compile(loss='mean_squared_error', optimizer='Adam')
history = model.fit(InsSeq, OutsSeq, epochs=50, validation_split=0.1)

model.save('data/modelrnn')
exit()

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
