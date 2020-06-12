'''
RNN modeling of motion data
'''
import numpy as np
import tensorflow as tf
from tensorflow import keras
import time

OutsSeq = np.load('data/OutsSeq.npy')
InsSeq = np.load('data/InsSeq.npy')

batch_size = OutsSeq.shape[0]
nsteps = OutsSeq.shape[1]

# RNN model
print(OutsSeq[0, ...].shape)
model = keras.models.Sequential([
    keras.layers.GRU(400, return_sequences=True, input_shape=[None, 2]),
    #keras.layers.GRU(10, return_sequences=True),
    #keras.layers.GRU(10, return_sequences=True),
    #keras.layers.GRU(10, return_sequences=True),
    #keras.layers.GRU(10, return_sequences=True),
    keras.layers.GRU(400),
    keras.layers.Dense(OutsSeq[0, ...].shape[0])
])

model.compile(loss='mean_squared_error', optimizer='Adam')
history = model.fit(InsSeq, OutsSeq, epochs=15, validation_split=0.2)

model.save('data/modelRNN')

# predictions
'''
for i in range(300, InsSeq.shape[0]):
    pred = model.predict(InsSeq[i, np.newaxis, :, :])
    print('\n{} input:{}'.format(i, InsSeq.shape))
    for j in range(InsSeq.shape[1]):
        value = InsSeq[i, j]
        print('({:5.2f},{:5.2f}) '.format(value[0], value[1]), end=' ')
    print('\npred {}:'.format(pred.shape))
    for j in range(pred.shape[1]):
        value = pred[0, j]
        print('{:5.2f}'.format(value), end= ' ')
    print('\noutput: {}'.format(OutsSeq.shape))
    for j in range(OutsSeq.shape[1]):
        value = OutsSeq[i, j, 0]
        print('{:5.2f}'.format(value), end=' ')
    time.sleep(.10)
'''