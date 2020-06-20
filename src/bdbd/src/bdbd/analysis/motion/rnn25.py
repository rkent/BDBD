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
    keras.layers.SimpleRNN(40, return_sequences=True, input_shape=(None, 2)),
    #keras.layers.Dense(10),
    #keras.layers.Dense(200),
    #keras.layers.Dense(100),
    #keras.layers.Dense(100),
    keras.layers.Dense(3),
])
print(model.summary())
#input('?')

model.compile(loss='mean_squared_error', optimizer='Adam')
history = model.fit(InsSeq, OutsSeq, epochs=10, validation_split=0.1)

#model.save('data/modelrnn10')
exit()
