'''
VGG in modular keras backend
'''

import keras
from keras.layers import Dense, Conv2D, Dropout, Input, Activation, Flatten
from keras.layers.convolutional import Convolution2D, MaxPooling2D, ZeroPadding2D, AveragePooling2D
from keras.optimizers import Adam
from keras.regularizers import l2
from keras.losses import categorical_crossentropy



def predictionMC():
    pass


def fer_vgg(input_shape=(48,48,1), input_classes= 7):
    inputs = keras.Input(shape=input_shape)
    # Conv Block 1
    x = Conv2D(64, (3,3), activation = 'relu', padding='same')(inputs)
    x = Dropout(0.25)(x, training = True)
    x = Conv2D(64, (3,3), activation = 'relu', padding = 'same')(inputs)
    x = MaxPooling2D(pool_size = (2,2) , strides = (2,2))(x)
    x = Dropout(0.25)(x, training = True)

    # Conv Block 2
    x = Conv2D(128, (3,3), activation='relu', padding='same')(x)
    x = Dropout(0.25)(x, training = True)
    x = Conv2D(128, (3,3), activation='relu', padding='same', kernel_regularizer=l2(0.001))(x)
    x = MaxPooling2D((2,2), strides=(2,2))(x)
    x = Dropout(0.25)(x, training = True)

    # Conv Block 3
    x = Conv2D(256, (3, 3), padding='same', activation ='relu')(x)
    x = Dropout(0.25)(x, training = True)
    x = Conv2D(256, (3, 3), padding='same', activation = 'relu', kernel_regularizer=l2(0.001))(x)
    x = Dropout(0.25)(x, training = True)
    x = Conv2D(256, (3, 3), padding='same', activation = 'relu', kernel_regularizer=l2(0.001))(x)
    x = AveragePooling2D(pool_size = (2, 2), strides=(2, 2))(x)
    x = Dropout(0.25)(x, training=True)

    # Conv Block 4
    x = Conv2D(512, (3, 3), padding = 'same', activation = 'relu')(x)
    x = Dropout(0.25)(x, training = True)
    x = Conv2D(512, (3, 3), padding = 'same', activation = 'relu')(x)
    x = Dropout(0.25)(x, training = True)
    x = Conv2D(512, (3, 3), padding = 'same', activation = 'relu', kernel_regularizer = l2(0.001))(x)
    x = Dropout(0.25)(x, training = True)
    x = Conv2D(512, (3, 3), padding = 'same', activation = 'relu', kernel_regularizer = l2(0.001))(x)
    x = AveragePooling2D(pool_size = (2,2), strides=(2,2))(x)
    x = Dropout(0.25)(x, training=True)

    # Conv Block 5
    x = Conv2D(512, (3, 3), padding = 'same', activation = 'relu')(x)
    x = Dropout(0.25)(x, training = True)
    x = Conv2D(512, (3, 3), padding = 'same', activation = 'relu')(x)
    x = Dropout(0.25)(x, training = True)
    x = Conv2D(512, (3, 3), padding = 'same', activation = 'relu', kernel_regularizer = l2(0.001))(x)
    x = Dropout(0.25)(x, training = True)
    x = Conv2D(512, (3, 3), padding = 'same', activation = 'relu', kernel_regularizer = l2(0.001))(x)
    x = AveragePooling2D(pool_size = (2,2), strides=(2,2))(x)
    x = Dropout(0.25)(x, training=True)

    # FC Layers
    x = Flatten()(x)
    #x = Dense(4096, activation='relu')(x)
    #x = Dropout(0.25)(x, training = True)
    x = Dense(512, activation='relu')(x)
    x = Dropout(0.25)(x, training=True)
    x = Dense(256, activation='relu')(x)
    x = Dropout(0.25)(x)
    output = Dense(7, activation='softmax')(x)

    model = keras.Model(inputs, output)
    return model

if __name__ == "__main__":
    model = fer_vgg()
    model.summary()
