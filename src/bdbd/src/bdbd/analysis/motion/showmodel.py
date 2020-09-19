import tensorflow
import tensorflow.keras as keras

model = keras.models.load_model('data/modelRnnFortieth15a')
print(model.summary())

for layer in model.layers[:]:
    layer.trainable = False

print(model.summary())
