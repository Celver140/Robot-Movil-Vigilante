import os
import string
import numpy as np
import tensorflow as tf
from tensorflow import keras
import re

landmarks = []
labels = []

with open('products.csv', 'r') as file:
    index = 0
    for line in file:
        line = line.split(',')
        landmarks.append(line[0:42])
        index = index+1
        result = re.search(r'(.+)(?:\\n|$)',line[42])
        if(result.group(0) == 40):
            print(index)
        labels.append(result.group(0))

# Define a simple sequential model
def create_model():
    model = tf.keras.models.Sequential([
        keras.layers.Dense(128, activation='relu', input_shape=(42,)),
        keras.layers.Dense(64, activation='relu'),
        keras.layers.Dense(64, activation='tanh'),
        keras.layers.Dropout(0.2),
        keras.layers.Dense(6, activation = 'softmax')
    ])

    model.compile(optimizer='adam',
                loss=tf.losses.SparseCategoricalCrossentropy(from_logits=True),
                metrics=[tf.metrics.SparseCategoricalAccuracy()])

    return model

# Create a basic model instance
model = create_model()
landmarks_np = np.array(landmarks, dtype = float)
labels_np = np.array(labels, dtype = int)

model.fit(landmarks_np, labels_np, epochs=10, batch_size=32, validation_split=0.25)
# Display the model's architecture
model.summary()

model.save('modelo.keras')
