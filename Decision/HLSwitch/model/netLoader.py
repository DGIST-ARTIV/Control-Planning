#Light VGG shinkansan
import tensorflow as tf
from keras.models import Sequential
from keras.optimizers import SGD,Adadelta
from keras.layers.core import Flatten, Dense, Dropout
from keras.layers.convolutional import Convolution2D, MaxPooling2D, ZeroPadding2D,AveragePooling2D
from keras.layers import Input, Dense, Dropout, Activation, Flatten, Conv2D, BatchNormalization
from keras.layers.advanced_activations import PReLU
# import the necessary packages
from keras.applications import ResNet50
from keras.applications import InceptionV3
from keras.applications import Xception # TensorFlow ONLY
from keras.applications import VGG16
from keras.applications import VGG19
from keras.applications import imagenet_utils
from keras.applications.inception_v3 import preprocess_input
from keras.preprocessing.image import img_to_array
from keras.preprocessing.image import load_img
import numpy as np
from keras.applications.xception import Xception, preprocess_input
from keras.optimizers import Adam
from keras.regularizers import l2
from keras.preprocessing import image
from keras.losses import categorical_crossentropy
from keras.layers import Dense, GlobalAveragePooling2D
from keras.models import Model
from keras.utils import to_categorical
from keras.callbacks import ModelCheckpoint
import keras
gpu_options = tf.GPUOptions(allow_growth=True)
sess = tf.Session(config=tf.ConfigProto(gpu_options=gpu_options))
keras.backend.tensorflow_backend.set_session(sess)
import math
import argparse
import cv2

McDropout = False

def fer_vgg(input_shape=(48, 48,1), input_classes=7):
    model1 = Sequential()
    #block 1
    model1.add(Conv2D(64, (3,3),activation='relu', padding='same',input_shape=input_shape))
    model1.add(Conv2D(64, (3,3), activation='relu', padding = 'same'))
    model1.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))
    #model1.add(Dropout(0.25, training=McDropout))
    #block 2
    model1.add(Conv2D(128, (3, 3), padding='same'))
    model1.add(PReLU(init='zero', weights=None))
    model1.add(Conv2D(128, (3,3), padding='same'))
    model1.add(PReLU(init='zero', weights=None))
    model1.add(MaxPooling2D((2,2), strides = (2,2), name = 'poo2_2'))
    #model1.add(Dropout(0.25, training=McDropout))
    #block 3
    model1.add(Conv2D(256, (3, 3), padding='same', activation='relu'))
    model1.add(Conv2D(256, (3, 3), padding='same'))
    model1.add(Conv2D(256, (3,3), padding='same', activation='relu', kernel_regularizer=l2(0.001)))
    model1.add(AveragePooling2D(pool_size=(2, 2), strides=(2, 2)))
    #model1.add(Dropout(0.25, training=McDropout))
    #block 4

    model1.add(Conv2D(512, (3, 3), padding='same', activation='relu'))
    model1.add(Conv2D(512, (3, 3), padding='same', activation = 'relu'))
    model1.add(Conv2D(512, (3,3), padding='same', activation= 'relu'))
    model1.add(Conv2D(512, (3, 3), padding='same', activation='relu', kernel_regularizer=l2(0.001)))
    model1.add(AveragePooling2D(pool_size=(2, 2), strides=(2, 2)))
    #model1.add(Dropout(0.25, training=True))
    #block 5
    model1.add(Conv2D(512, (3,3), padding='same', activation='relu'))
    model1.add(Conv2D(512, (3,3), padding='same', activation='relu'))
    model1.add(Conv2D(512, (3,3), padding='same', activation='relu'))
    model1.add(Conv2D(512, (3,3), padding='same', activation='relu', kernel_regularizer=l2(0.001)))
    model1.add(AveragePooling2D(pool_size=(2, 2), strides=(2, 2)))
    #model1.add(Dropout(0.25, training=McDropout))

    # Fc layer
    model1.add(Flatten())
    model1.add(Dense(4096))
    model1.add(PReLU(init='zero', weights=None))
    #model1.add(Dropout(0.5, training=McDropout))
    model1.add(Dense(1024))
    model1.add(PReLU(init='zero', weights=None))
    #model1.add(Dropout(0.5, training=McDropout))
    model1.add(Dense(512))
    model1.add(PReLU(init='zero', weights=None))
    #model1.add(Dropout(0.3, training=McDropout))
    model1.add(Dense(input_classes))
    model1.add(Activation('softmax'))

    return model1

def hlswitch():

    # Convolution Neural Networks (CNN)
    model = Sequential()

    model.add(Conv2D(8, kernel_size=(3,3), padding='same', input_shape = (200,200,1)))
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(3, 3)))

    model.add(Conv2D(16, kernel_size=(3,3), padding='same'))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Conv2D(32, kernel_size=(3,3), padding='same'))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Conv2D(32, kernel_size=(3,3), padding='same'))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D(pool_size=(2, 2)))

    model.add(Flatten())
    model.add(Dense(1, activation='sigmoid'))
    return model



def sample(model, input_shape, classes):
    models = {"vgg16" : VGG16, "vgg19":VGG19, "inception" : InceptionV3, "xception" : Xception, "resnet" : ResNet50}
    if model in models:

        network = models[model]
        model = network(include_top = False, input_shape=input_shape)

        x = model.output
        x = GlobalAveragePooling2D()(x)
        x = Dense(1024, activation='relu')(x)
        prediction = Dense(classes, activation='softmax')(x)
        Fmodel = Model(inputs=model.inputs, outputs=prediction)

        Fmodel.compile(loss = categorical_crossentropy, optimizer=Adam(lr=1e-4),metrics=['accuracy'])
        Fmodel.summary()
        return Fmodel

if __name__ == "__main__":
    model1 = fer_vgg((48,48,1), 7)
    model1.summary()

    models = {"vgg16" : VGG16, "vgg19":VGG19, "inception" : InceptionV3, "xception" : Xception, "resnet" : ResNet50}
