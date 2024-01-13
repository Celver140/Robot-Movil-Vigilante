#!/usr/bin/env python
# -*- coding: utf-8 -*-

# ------------------------------------------------------------------------------------------------
# Este nodo simula el input de la camara del robot real para la simulación
# ------------------------------------------------------------------------------------------------

import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image

# ------------------------------------------------------------------------------------------------
# TOPICS Y VARIABLES GLOBALES
# ------------------------------------------------------------------------------------------------

TOPIC_IMG = '/camera/rgb/image_raw'
frame = None

class WebcamPublisher:
    def __init__(self):
        rospy.init_node('webcam_publisher', anonymous=True)
        self.bridge = cv_bridge.CvBridge()  # Necesario para la transformación de mensaje de ROS a imagen
        self.image_pub = rospy.Publisher(TOPIC_IMG, Image, queue_size=10)   # Publicador para transmitir la imagen
        self.capture = cv2.VideoCapture(0)  # 0 para la webcam por defecto

    def publish_frame(self):
        ret, frame = self.capture.read()    # Obtención de la imagen
        if ret:
            try:
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")    # Transformar de mensaje a imagen
                self.image_pub.publish(ros_image)                       # Publicar en nodo correspondiente
            except cv_bridge.CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.publish_frame()
            rate.sleep()

        self.capture.release()

if __name__ == '__main__':
    try:
        webcam_publisher = WebcamPublisher()
        webcam_publisher.run()
    except rospy.ROSInterruptException:
        pass
