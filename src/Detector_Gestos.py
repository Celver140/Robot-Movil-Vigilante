#!/usr/bin/env python3.10

# -*- coding: utf-8 -*-
# from __future__ import print_function

# ------------------------------------------------------------------------------------------------
# Este nodo detecta manos y publica la posicion de la muñeca
# ------------------------------------------------------------------------------------------------

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, String, Int32
import mediapipe

import tensorflow as tf
from tensorflow import keras
import copy
import itertools
import numpy as np
import math

# ------------------------------------------------------------------------------------------------
# TOPICS Y VARIABLES GLOBALES
# ------------------------------------------------------------------------------------------------

# Redirigir stdout y stderr a /dev/null

TOPIC_IMG= '/camera/rgb/image_raw'
TOPIC_WRIST = '/wrist_position'
TOPIC_GESTURE = '/gestos'
TOPIC_FINGERCOUNT = '/num_waypoint'
CLASSNAMES = ['puño', 'palma', 'seguir', 'waypoint', 'okay', 'cancel']
frame = []

# ------------------------------------------------------------------------------------------------
# DETECCIÓN DE MANOS
# ------------------------------------------------------------------------------------------------

class WristDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(TOPIC_IMG, Image, self.image_callback)

        self.drawingModule = mediapipe.solutions.drawing_utils
        self.handsModule = mediapipe.solutions.hands

        #publisher
        self.pub = rospy.Publisher(TOPIC_WRIST, Int32MultiArray, queue_size=5)
        self.pub_gesture = rospy.Publisher(TOPIC_GESTURE, String, queue_size=5) 
        self.pub_fingercount = rospy.Publisher(TOPIC_FINGERCOUNT, Int32, queue_size=5) 

        self.model = keras.models.load_model("modelo.keras", compile=False)
        
    def image_callback(self, msg):
        global frame
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8") 

        with self.handsModule.Hands(static_image_mode=False, min_detection_confidence=0.75, min_tracking_confidence=0.75, max_num_hands=1) as hands:
            results = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            if results.multi_hand_landmarks:
                frameWidth, frameHeight = frame.shape[:2]
                detected = 0
                for hand_index, handLandmarks in enumerate(results.multi_hand_landmarks):
                    # 1. Dibujar mano
                    self.drawingModule.draw_landmarks(frame, handLandmarks, self.handsModule.HAND_CONNECTIONS)

                    wristLandmark = handLandmarks.landmark[self.handsModule.HandLandmark.WRIST]
                    wristCoordinates = self.drawingModule._normalized_to_pixel_coordinates(wristLandmark.x, wristLandmark.y, frameWidth, frameHeight)    

                    # Obtener diferencia de las coordeandas de la mano respecto al centro de la imagen 
                    if wristCoordinates[0] is not None:
                        dif = wristCoordinates[0] - int(frameWidth/2)
                        dif = np.int32(dif)

                        # Obtener area del cuadrado delimitador
                        bbox = self.calculate_bounding_box(handLandmarks, frameWidth, frameHeight)
                        bbox_width = bbox[1][0] - bbox[0][0]
                        bbox_height = bbox[1][1] - bbox[0][1]
                        area = int(bbox_height*bbox_width) # Varía entre 110000 y 2500. 

                        #print("Dif: ", dif, "Area: ", area)

                        int_array_msg = Int32MultiArray()
                        int_array_msg.data = [dif, area]  
                        
                        self.pub.publish(int_array_msg)       

                    landmark_list = self.calc_landmark_list(frame, handLandmarks)
                    landmark_coord = self.pre_process_landmark(landmark_list)
                    landmark_dist = self.landmark_distance(landmark_list)
                    prediction = self.model.predict([landmark_coord])
                    Gesto = CLASSNAMES[np.argmax(prediction)]
                    print(Gesto)
                    self.pub_gesture.publish(Gesto)

                    # Contabilizar los dedos que hay
                    if landmark_dist[8] > 0.8:
                        detected += 1                        
                    if landmark_dist[12] > 0.8:
                        detected += 1
                    if landmark_dist[16] > 0.8:
                        detected += 1
                    if landmark_dist[20] > 0.7:
                        detected += 1
                    if (landmark_dist[4] > 0.95 and detected == 0) or (landmark_dist[4] > 0.7 and detected != 0):
                        detected += 1

                    self.pub_fingercount.publish(detected)  #publicar la cuenta de dedos
            else:
                self.pub_gesture.publish('none')
                self.pub_fingercount.publish(0)

        cv2.imshow("Gesture Detector", frame)
        cv2.waitKey(3)

    def calc_landmark_list(self, image, landmarks):
        image_width, image_height = image.shape[1], image.shape[0]

        landmark_point = []

        # Keypoint
        for _, landmark in enumerate(landmarks.landmark):
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)
            # landmark_z = landmark.z

            landmark_point.append([landmark_x, landmark_y])

        return landmark_point

    def pre_process_landmark(self, landmark_list):
        temp_landmark_list = copy.deepcopy(landmark_list)

        # Convert to relative coordinates
        base_x, base_y = 0, 0
        for index, landmark_point in enumerate(temp_landmark_list):
            if index == 0:
                base_x, base_y = landmark_point[0], landmark_point[1]

            temp_landmark_list[index][0] = temp_landmark_list[index][0] - base_x
            temp_landmark_list[index][1] = temp_landmark_list[index][1] - base_y

        # Convert to a one-dimensional list
        temp_landmark_list = list(
            itertools.chain.from_iterable(temp_landmark_list))

        return temp_landmark_list
        
    def calculate_bounding_box(self, handLandmarks, frameWidth, frameHeight):
        x_min = min([landmark.x for landmark in handLandmarks.landmark])
        x_max = max([landmark.x for landmark in handLandmarks.landmark])
        y_min = min([landmark.y for landmark in handLandmarks.landmark])
        y_max = max([landmark.y for landmark in handLandmarks.landmark])

        return [(x_min * frameWidth, y_min * frameHeight), (x_max * frameWidth, y_max * frameHeight)]

    def landmark_distance(self, landmark_list):
        temp_landmark_list = copy.deepcopy(landmark_list)

        # Convert to relative coordinates
        base_x, base_y = 0, 0
        for index, landmark_point in enumerate(temp_landmark_list):
            if index == 0:
                base_x, base_y = landmark_point[0], landmark_point[1]

            temp_landmark_list[index] = math.sqrt(pow(temp_landmark_list[index][0] - base_x,2) + pow(temp_landmark_list[index][1] - base_y,2))
        

        # Normalization
        max_value = max(list(map(abs, temp_landmark_list)))

        def normalize_(n):
            return n / max_value

        temp_landmark_list = list(map(normalize_, temp_landmark_list))

        return temp_landmark_list
# ------------------------------------------------------------------------------------------------
# MAIN
# ------------------------------------------------------------------------------------------------

rospy.init_node('wrist_detector')
cd  = WristDetector()
rospy.spin()
