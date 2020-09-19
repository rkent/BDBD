# test of tensorflow usage
import tensorflow as tf

x = tf.Variable(5.0)
dt = 0.1
xsum = tf.Variable(0.0, trainable=True)

with tf.GradientTape() as tape:
    t = 0.0
    for count in range(0, 20):
        y = x**2
        xsum = xsum + dt * y
        print(t, y.numpy(), xsum.numpy())
        t += dt

print(tape.gradient(xsum, x)) # -> 75.0
print(xsum)
