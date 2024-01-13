#!/usr/bin/env python

# -*- coding: utf-8 -*-
# from __future__ import print_function

# ------------------------------------------------------------------------------------------------
# Nodo para detectar personas
# ------------------------------------------------------------------------------------------------

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import mediapipe as mp

# ------------------------------------------------------------------------------------------------
# TOPICS Y VARIABLES GLOBALES
# ------------------------------------------------------------------------------------------------

# Redirigir stdout y stderr a /dev/null

TOPIC_IMG= '/camera/rgb/image_raw'
TOPIC_PERSONA = '/persona_detectada'

frame = []

# ------------------------------------------------------------------------------------------------
# DETECCIÓN DE PERSONAS
# ------------------------------------------------------------------------------------------------

class PersonaDetector:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(TOPIC_IMG, Image, self.image_callback)
        
        #publisher
        self.pub = rospy.Publisher(TOPIC_PERSONA, Int32MultiArray, queue_size=5) 
        self.position_msg = Int32MultiArray()

        # MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()
        
    def image_callback(self, msg):
        global frame
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8") 
        frameWidth, frameHeight = frame.shape[:2]

        results = self.pose.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        if results.pose_landmarks:
            # Dibujar marcadores de pose
            # mp.solutions.drawing_utils.draw_landmarks(frame, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

            # Landmarks de los hombros
            left_shoulder = results.pose_landmarks.landmark[11]
            right_shoulder = results.pose_landmarks.landmark[12]

            # Convertir coordenadas a píxeles
            ls_x, ls_y = int(left_shoulder.x * frame.shape[1]), int(left_shoulder.y * frame.shape[0])
            rs_x, rs_y = int(right_shoulder.x * frame.shape[1]), int(right_shoulder.y * frame.shape[0])

            # Dibujar círculo promedio de ambos
            pos_x = int((ls_x + rs_x)/2)
            pos_y = int((ls_y + rs_y)/2)

            cv2.circle(frame, (pos_x, pos_y), 10, (0, 0, 255), -1)

            # Publicar diferencia con el centro de la imagen
            dif = pos_x - int(frameWidth/2)
            area = 1
            self.position_msg.data = [dif, area] # se envían las coordenadas en pixeles de la imagen 
            self.pub.publish(self.position_msg)

        cv2.imshow("Human Detector", frame)
        cv2.waitKey(3)

        

# ------------------------------------------------------------------------------------------------
# MAIN
# ------------------------------------------------------------------------------------------------

rospy.init_node('persona_detector')
cd  = PersonaDetector()
rospy.spin()    
