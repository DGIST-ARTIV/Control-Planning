## Train py for HL Switch
# Binary Classification
# 2020.09.12. Shin
# v04

import numpy as np
import matplotlib.pyplot as plt
from keras.datasets import mnist
from keras.models import Sequential
from keras.callbacks import ModelCheckpoint
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.layers.convolutional import Convolution2D, MaxPooling2D
from keras.preprocessing.image import ImageDataGenerator
from keras.utils import np_utils
from keras.utils import multi_gpu_model
#from keras.regularizers import l2, activity_l2
from keras.optimizers import SGD, RMSprop
import matplotlib.pyplot as plt
import h5py
import keras
from keras.callbacks import EarlyStopping
import model.netLoader as netLoader
import tensorflow as tf
import os
session_config = tf.ConfigProto()
session_config.gpu_options.allow_growth = True
session = tf.Session(config=session_config)

use_date_gen = True
batch_size = 32

np.random.seed(77778)  # for reproducibility


def visual(hist):
    loss_ax = plt.subplot()
    acc_ax = loss_ax.twinx()

    loss_ax.plot(hist.history['loss'], 'y', label='train loss')
    loss_ax.plot(hist.history['val_loss'], 'r', label='val loss')

    acc_ax.plot(hist.history['acc'], 'b', label='train acc')
    acc_ax.plot(hist.history['val_acc'], 'g', label='val acc')

    loss_ax.set_xlabel('epoch')
    loss_ax.set_ylabel('loss')
    acc_ax.set_ylabel('accuray')

    loss_ax.legend(loc='upper left')
    acc_ax.legend(loc='lower left')

    plt.show()
    pass


# Load the scaled data, both pixels and labels
X_train = np.load('./data/Scaled_6.npy')
Y_tr_labels = np.load('./data/labeld_6.npy')

# reshape the given pixels into 48 X 48 images
if use_date_gen:

    train_datagen = ImageDataGenerator(
            rescale=1./255,
            shear_range=0.2,
            zoom_range=0.2,
            horizontal_flip=True)


    test_datagen = ImageDataGenerator(rescale=1./255)

    train_generator = train_datagen.flow_from_directory(
            './TrainData/',
            color_mode = 'grayscale',
            target_size=(200, 200),
            batch_size=batch_size,
            class_mode='binary')

else:

    shapex, shapey = 320, 180
    X_train = X_train.reshape(X_train.shape[0], shapey, shapex, 1)
    # convert labels to one-hot-encoding
    Y_tr_labels = np_utils.to_categorical(Y_tr_labels).astype('int8')




# define the model 32 filters in first convolution layer followed by a max pooling and dense layer with dropout (50%)
tb_hist = keras.callbacks.TensorBoard(log_dir='./graph', histogram_freq=0, write_graph=True, write_images=True)
early_stopping = EarlyStopping(monitor='val_loss', patience = 20)
MODEL_SAVE_FOLDER_PATH = './models/'
if not os.path.exists(MODEL_SAVE_FOLDER_PATH):
  os.mkdir(MODEL_SAVE_FOLDER_PATH)

model_path = MODEL_SAVE_FOLDER_PATH + '{epoch:02d}-{val_loss:.4f}.hdf5'

cb_checkpoint = ModelCheckpoint(filepath=model_path, monitor='val_loss',
                                verbose=1, save_best_only=True)

################################### model1
#model = netLoader.fer_vgg(input_shape=(180,320,1), input_classes=2)
model = netLoader.hlswitch()
ans_dict = {0:'GPS', 1:'VISION'}

# training the model with cross sgd and nesterov momentum
#model = multi_gpu_model(model, gpus=3)
sgd = SGD(lr=0.055, decay=1e-6, momentum=0.9, nesterov=True)
optm = RMSprop(lr=0.004, rho=0.9, epsilon=1e-08, decay=0.0)
model.compile(loss='binary_crossentropy', optimizer=optm, metrics=['accuracy', 'loss'])
hist = ''
if use_date_gen:
    hist = model.fit_generator(
            train_generator,
            steps_per_epoch=2000 // batch_size,
            epochs=100)

else:
    hist = model.fit(X_train, Y_tr_labels, validation_split=0.1, batch_size=batch_size, nb_epoch=10, callbacks=[tb_hist, early_stopping, cb_checkpoint])

print("Training Complete!")
# save the model weights

model.save('./models/final_model_new_4.hdf5')
del model

visual(hist)
