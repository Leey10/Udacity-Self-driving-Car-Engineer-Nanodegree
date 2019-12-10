from keras.models import Sequential
from keras.layers import *

# Define the model structure
def network():
    model = Sequential()
    # Normalization
    model.add(Lambda(lambda x: (x / 127.5) - 1.0, input_shape = (70,160,3)))
    
    # Convlutional Layer 1
    model.add(Conv2D(filters=24, kernel_size=5, strides=(2,2)))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    
    # Convlutional Layer 2
    model.add(Conv2D(filters=36, kernel_size=5, strides=(2,2)))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    
    # Convlutional Layer 3
    model.add(Conv2D(filters=48, kernel_size=5, strides=(2,2)))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    
    # Convlutional Layer 4
    model.add(Conv2D(filters=64, kernel_size=3, strides=(1,1)))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    
    # Convlutional Layer 5
    model.add(Conv2D(filters=64, kernel_size=3, strides=(1,1)))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    
    # Flatten Layer
    model.add(Flatten())
    
    # Fully Connected Layer 1
    model.add(Dense(100))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    
    # Fully Connected Layer 2
    model.add(Dense(50))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    
    # Fully Connected Layer 3
    model.add(Dense(10))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    
    # Output Layer
    model.add(Dense(1))
    
    # Configure training method
    model.compile(loss='mse',
              optimizer='adam',
              metrics=['accuracy'])
    
    return model