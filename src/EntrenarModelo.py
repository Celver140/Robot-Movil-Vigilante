#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import string
import numpy as np
import tensorflow as tf
from tensorflow import keras
import re

# Función para crear el modelo base
def create_model(): 
    model = tf.keras.models.Sequential([
        keras.layers.Dense(128, activation='relu', input_shape=(42,)),  # Capas ocultas
        keras.layers.Dense(64, activation='relu'),
        keras.layers.Dense(64, activation='tanh'),
        keras.layers.Dropout(0.2),          # Desactivación de neuronas para no sobreentrenar
        keras.layers.Dense(6, activation = 'softmax')   # Capa de salida
    ])

    # Compilar modelo para su entrenamiento
    model.compile(optimizer='adam',
                loss=tf.losses.SparseCategoricalCrossentropy(from_logits=True),
                metrics=[tf.metrics.SparseCategoricalAccuracy()])

    return model


if __name__ == '__main__':
    
    # Declaración de variables
    landmarks = []  
    labels = []

    # Lectura de la base de datos
    with open('products.csv', 'r') as file:
        for line in file:   # Para cada entrada de la base de datos
            line = line.split(',')  # Obtener todos los valores y separarlos en las listas correspondientes
            landmarks.append(line[0:42])
            labels.append(line[42])

    # Crear instancia del modelo
    model = create_model()
    landmarks_np = np.array(landmarks, dtype = float)   # Adaptar las listas a numpy
    labels_np = np.array(labels, dtype = int)

     # Entrenar modelo configuración del entrenamiento
    model.fit(landmarks_np, labels_np, epochs=10, batch_size=32, validation_split=0.25)

    # Mostrar arquitectura
    model.summary()

    # Guardar modelo
    model.save('modelo.keras')
