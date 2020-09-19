# demo of using tensorflow gradients to optimize a future pose

import os
os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

import tensorflow as tf
import tensorflow.keras as keras
import math
import time

class Object():
    pass

def v_limit(v, mn, mx):
    value = max(mn, min(mx, v.numpy()))
    v.assign(value)

def getLoss(predictions):
    x = 0.0
    y = 0.0
    xt = .15
    yt = .05
    theta = 0.0
    deltat = 0.025
    for prediction in predictions[0]:
        vx = prediction[0]
        vy = prediction[1]
        omega = prediction[2]
        theta = theta + deltat * omega
        cosine = tf.math.cos(theta)
        sine = tf.math.sin(theta)
        map_vx = vx * cosine - vy * sine
        map_vy = vx * sine + vy * cosine
        x = x + deltat * map_vx
        y = y + deltat * map_vy

    loss = (x - xt)**2 + (y - yt)**2
    return loss, x, y, theta

def predict(model, inputs, tmax, deltat):

    result = Object()
    # create the inputses
    t = 0.0
    vmax = 0.8
    speeds = []
    start = time.time()
    with tf.GradientTape(watch_accessed_variables=False, persistent=True) as tape:
        for input in inputs:
            tape.watch(input)
        rho = inputs[0]
        tau = inputs[1]

        left = tf.math.minimum(tf.constant(vmax), vmax * (1. + 2 * rho))
        right = tf.math.minimum(tf.constant(vmax), vmax * (1. - 2 * rho))
        
        while t < tmax + deltat:
            t += deltat
            tzero = 0.5 * (1. - tf.math.sign(t - tau))
            factor = (tau - t) / tau
            speeds.append((tzero * factor * left, tzero * factor * right))

        tf_speeds = tf.convert_to_tensor(speeds)
        tf_speedses = tf.expand_dims(tf_speeds, axis=0)
        predictions = model(tf_speedses)
        loss, x, y, theta = getLoss(predictions)
        #print(tf_speeds.numpy())

    gradients = tape.gradient(loss, inputs)
    print('predictions delta: {:7.4f}'.format(time.time() - start))
    #print('after opt: {}'.format(inputs))
    result.gradients = gradients
    result.loss = loss
    result.x = x
    result.y = y
    result.theta = theta
    return result

'''
def loss(result):
    xt = 0.20
    yt = 0.0
    thetat = 0.0
    return ((result.xs[-1] - xt)**2 + (result.ys[-1] - yt)**2 + (result.thetas[-1] - thetat)**2)
'''

DT = 0.025
TMAX = 2.0
model = keras.models.load_model('data/modelRnnFortieth15a')
rho = tf.Variable(.1, name='rho')
tau = tf.Variable(1.0, name='tau')

inputs = (rho, tau)
opt = keras.optimizers.Adam(learning_rate=0.05)

for count in range(0, 5):
    start = time.time()
    result = predict(model, inputs, tmax=TMAX, deltat=DT)
    print('{:4.0f} loss: {:7.5f} x: {:7.5f} y: {:7.5f} theta: {:7.5f} rho: {:7.5f} tau: {:7.5f}'.format(count, result.loss, result.x, result.y, result.theta, *[value.numpy() for value in inputs]))
    if result.loss < 0.00004:
        break
    opt.apply_gradients(zip(result.gradients, inputs))
    v_limit(rho, -1.0, 1.0)
    v_limit(tau, .1, TMAX)
    delta = time.time() - start
    print('delta: {:7.5f}'.format(delta))

    #print([v_a, v_b])
    #a -= gradients[0].numpy()
    #b -= gradients[1].numpy()

