#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QListWidget, QRadioButton
from PyQt5.QtGui import QPalette, QColor, QPixmap, QIcon
from PyQt5.QtCore import QTimer, Qt
import rospy, cv2, cv_bridge
import numpy as np

from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Bool 
from geometry_msgs.msg import Point, PoseArray, Pose
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker

# ------------------------------------------------------------------------------------------------
# TOPICS Y VARIABLES GLOBALES
# ------------------------------------------------------------------------------------------------

TOPIC_IMG= '/camera/rgb/image_raw'
TOPIC_WAYPOINT= '/waypoints'
TOPIC_ESTADOS= '/estados'
TOPIC_ALARMA= '/alarma'

# ------------------------------------------------------------------------------------------------
# Interfaz
# ------------------------------------------------------------------------------------------------

class DarkThemeApp(QWidget):
    def __init__(self):
        super().__init__()

        self.init_ui()
        rospy.init_node("Interfaz")
        
        rospy.Subscriber(TOPIC_WAYPOINT, PoseArray, self.waypoint_callback) #Suscriptores para la recepción de cambios en la máquina de estados
        rospy.Subscriber(TOPIC_ESTADOS, String, self.sel_menu)

        self.pub_alarm = rospy.Publisher(TOPIC_ALARMA, Bool, queue_size=10) # Publicador para determinar si el robot debe tener la alarma preparada
        self.bridge = cv_bridge.CvBridge()  # Necesario para transformar de mensaje de ros a imagen


    def init_ui(self):

        # -------------------------------------------------------------------------------------------------
        # Configuración de la UI y de los colores
        # -------------------------------------------------------------------------------------------------

        # Configuración del diseño y la paleta de colores
        self.setWindowTitle('Robot vigilante')                                  # título de la ventana
        self.setFixedSize(750, 775)                                             # tamaño de la ventana
        self.setStyleSheet("background-color: #2E2E2E; color: #FFFFFF;")        # definición de algunos colores

        # Configurar la paleta de colores para el tema oscuro
        palette = QPalette()                                                    # objeto para personalizar colores
        palette.setColor(QPalette.Window, QColor(45, 45, 45))
        palette.setColor(QPalette.WindowText, QColor(255, 255, 255))
        palette.setColor(QPalette.Button, QColor(45, 45, 45))
        palette.setColor(QPalette.ButtonText, QColor(255, 255, 255))
        self.setPalette(palette)                                                # aplica la paleta de colores personalizada

        # Inicializar layout principal que contendrá a los otros layouts
        main_layout = QVBoxLayout(self)                                          
        self.setLayout(main_layout)
        main_layout.setSpacing(0)

        # -------------------------------------------------------------------------------------------------
        # Título inicial
        # -------------------------------------------------------------------------------------------------

        current_file_path = os.path.abspath(__file__)
        path = os.path.dirname(current_file_path) + '/icons/'     

        # Crear QLabel y ponerle la imagen como pixmap
        self.image_titulo = QLabel(self)                                         
        pixmap = QPixmap(path + 'titulo.png')
        self.image_titulo.setPixmap(pixmap)

        # -------------------------------------------------------------------------------------------------
        # Imágenes de los botones
        # -------------------------------------------------------------------------------------------------

        # # Crear QHBoxLayout para alinear horizontalmente los conjuntos de imagen y botón
        # botones_layout = QHBoxLayout()

        # # Crear y configurar los QRadioButtons y sus imágenes
        # botones = [
        #     ('Reposo', 'reposo.png'),
        #     ('Mapeado', 'mapeado.png'),
        #     ('Planificar', 'planificacion.png'),
        #     ('Navegacion', 'navigation.png')
        # ]

        # for nombre, imagen in botones:
        #     # Crear QVBoxLayout para cada conjunto de imagen y botón
        #     layout_individual = QVBoxLayout()

        #     # Cargar y configurar la imagen
        #     label_imagen = QLabel(self)
        #     pixmap = QPixmap(path + imagen)
        #     resized_pixmap = pixmap.scaled(40, 40, Qt.KeepAspectRatio)
        #     label_imagen.setPixmap(resized_pixmap)

        #     # Añadir imagen y botón al QVBoxLayout individual
        #     layout_individual.addWidget(label_imagen, alignment=Qt.AlignCenter)

        #     # Añadir el QVBoxLayout al QHBoxLayout principal
        #     botones_layout.addLayout(layout_individual)

        # -------------------------------------------------------------------------------------------------
        # Botones de los estados
        # -------------------------------------------------------------------------------------------------

        # Configurar los botones de radio en un diseño horizontal
        config = "QRadioButton::indicator{width : 30px;height : 30px;}QRadioButton{font : 20px Arial;}"
        
        radio_layout = QHBoxLayout()
        img_x = 40
        img_y = 40

        # Reposo ----------------------------------------------------------
        layout_individual1 = QVBoxLayout()
        layout_individual1.setSpacing(0)

        # Cargar y configurar la imagen
        label_imagen1 = QLabel(self)
        pixmap = QPixmap(path + 'reposo.png')
        resized_pixmap = pixmap.scaled(img_x, img_y, Qt.KeepAspectRatio)
        label_imagen1.setPixmap(resized_pixmap)

        self.radio_button1 = QRadioButton('Reposo', self)
        self.radio_button1.setStyleSheet(config)

        # Añadir imagen y botón al QVBoxLayout individual
        layout_individual1.addWidget(label_imagen1, alignment=Qt.AlignCenter)
        layout_individual1.addSpacing(20)
        layout_individual1.addWidget(self.radio_button1, alignment=Qt.AlignCenter)

        # Mapeado ----------------------------------------------------------
        layout_individual2 = QVBoxLayout()
        layout_individual2.setSpacing(0)

        # Cargar y configurar la imagen
        label_imagen2 = QLabel(self)
        pixmap = QPixmap(path + 'mapeado.png')
        resized_pixmap = pixmap.scaled(img_x, img_y, Qt.KeepAspectRatio)
        label_imagen2.setPixmap(resized_pixmap)

        self.radio_button2 = QRadioButton('Mapeado', self)
        self.radio_button2.setStyleSheet(config)

        # Añadir imagen y botón al QVBoxLayout individual
        layout_individual2.addWidget(label_imagen2, alignment=Qt.AlignCenter)
        layout_individual2.addSpacing(20)
        layout_individual2.addWidget(self.radio_button2, alignment=Qt.AlignCenter)

        # Planificacion ----------------------------------------------------------
        layout_individual3 = QVBoxLayout()
        layout_individual3.setSpacing(0)

        # Cargar y configurar la imagen
        label_imagen3 = QLabel(self)
        pixmap = QPixmap(path + 'planificacion.png')
        resized_pixmap = pixmap.scaled(img_x, img_y, Qt.KeepAspectRatio)
        label_imagen3.setPixmap(resized_pixmap)

        self.radio_button3 = QRadioButton('Planificación', self)
        self.radio_button3.setStyleSheet(config)

        # Añadir imagen y botón al QVBoxLayout individual
        layout_individual3.addWidget(label_imagen3, alignment=Qt.AlignCenter)
        layout_individual3.addSpacing(20)
        layout_individual3.addWidget(self.radio_button3, alignment=Qt.AlignCenter)

        # Navegacion ----------------------------------------------------------
        layout_individual4 = QVBoxLayout()
        layout_individual4.setSpacing(0)

        # Cargar y configurar la imagen
        label_imagen4 = QLabel(self)
        pixmap = QPixmap(path + 'navigation.png')
        resized_pixmap = pixmap.scaled(img_x, img_y, Qt.KeepAspectRatio)
        label_imagen4.setPixmap(resized_pixmap)

        self.radio_button4 = QRadioButton('Navegación', self)
        self.radio_button4.setStyleSheet(config)

        # Añadir imagen y botón al QVBoxLayout individual
        layout_individual4.addWidget(label_imagen4, alignment=Qt.AlignCenter)
        layout_individual4.addSpacing(20)
        layout_individual4.addWidget(self.radio_button4, alignment=Qt.AlignCenter)

        # Deshabilita la posibilidad de que los radiobuton sean seleccionados
        self.radio_button1.setDisabled(True)
        self.radio_button2.setDisabled(True)
        self.radio_button3.setDisabled(True)
        self.radio_button4.setDisabled(True)
        self.radio_button1.setChecked(True)

        #radio_group = [self.radio_button1, self.radio_button2, self.radio_button3, self.radio_button4]
        #for radio_button in radio_group:
            #radio_layout.addWidget(radio_button, alignment=Qt.AlignCenter) 
        
        radio_group = [layout_individual1, layout_individual2, layout_individual3, layout_individual4]
          
        for layout in radio_group:
            #layout.addStretch(5)
            layout.setAlignment(Qt.AlignTop)
            radio_layout.addLayout(layout)        

        # -------------------------------------------------------------------------------------------------
        # Alarma
        # -------------------------------------------------------------------------------------------------

        # Texto de los waypoints
        waypoints_text = QLabel('Waypoints:        ', self)
        waypoints_text.setStyleSheet("font-size: 20px;")

        # Texto de la alarma
        # label = QLabel('Alarma:', self)   
        # label.setStyleSheet("font-size: 20px;")

        self.alarma = False
        self.alarma_preparada = False

        # Botón de la alarma
        self.switch_button = QPushButton('Alarma \napagada', self)
        self.switch_button.setCheckable(True)
        self.switch_button.clicked.connect(self.on_switch_button_clicked)
        self.switch_button.setFixedSize(100, 50)

        # Texto de intruso
        self.led_label = QLabel('INTRUSO!',self)
        self.led_label.setAlignment(Qt.AlignCenter)
        self.led_label.setFixedSize(200, 50)  # Establecer el tamaño fijo del LED (circular)
        self.led_label.setStyleSheet("background-color:black;font : 20px Arial;color:black;")

        lists_alarma = QHBoxLayout()
        lists_alarma.addSpacing(40)
        lists_alarma.addWidget(waypoints_text) 

        #lists_alarma.addSpacing(50)
        #lists_alarma.addWidget(label) 

        #lists_alarma.addSpacing(50)  
        lists_alarma.addWidget(self.switch_button)

        lists_alarma.addSpacing(50)
        lists_alarma.addWidget(self.led_label)
        
        # -------------------------------------------------------------------------------------------------
        # Listas
        # -------------------------------------------------------------------------------------------------

        self.list_widget = QListWidget(self)
        self.list_widget.setFixedSize(300, 300)
   
        # Agregar las dos listas una al lado de la otra
        lists_layout = QHBoxLayout()
        lists_layout.addWidget(self.list_widget, alignment=Qt.AlignCenter)

        # -------------------------------------------------------------------------------------------------
        # Imagen del intruso
        # -------------------------------------------------------------------------------------------------
        
        # Crear un QLabel para mostrar la imagen redimensionada
        self.image_label = QLabel(self)
        lists_layout.addWidget(self.image_label, alignment=Qt.AlignCenter)
        
        # -------------------------------------------------------------------------------------------------
        # Añadir al layout
        # -------------------------------------------------------------------------------------------------
        main_layout.setAlignment(Qt.AlignTop)  
        
        main_layout.addWidget(self.image_titulo, alignment=Qt.AlignTop | Qt.AlignLeft)  # titulo
        main_layout.addSpacing(50)
              
        main_layout.addLayout(radio_layout) # botones estados
        main_layout.addSpacing(80)

        main_layout.addLayout(lists_alarma) # alarma
        main_layout.addLayout(lists_layout) # listas

        self.show()

        # Imprimir todos los elementos de list_trayect
        elementos_trayect = [self.list_trayect.item(i).text() for i in range(self.list_trayect.count())]
        print("Elementos en list_trayect:", elementos_trayect)

    def waypoint_callback(self, msg):
        
        self.mi_lista = []  # Limpiar la lista antes de agregar nuevos elementos
        self.list_widget.clear()  # Vaciar la QListWidget antes de agregar nuevos elementos
        i = 0
        for pose in msg.poses:
            i = i+1
            punto_str = ""
            punto_str = f"Punto {i}: X={round(pose.position.x, 3)}, Y={round(pose.position.y, 3)}"
            self.mi_lista.append(punto_str)

        for elemento in self.mi_lista:
            self.list_widget.addItem(str(elemento))

    # Para que actualice la eleccion del menu
    def sel_menu(self, msg):
        print(msg.data)
        if msg.data == "reposo":
            self.radio_button1.setChecked(True)
            self.set_alarma(Bool(False))
        elif msg.data == "mapeado":
            self.radio_button2.setChecked(True)
        elif msg.data == "planificacion":
            self.radio_button3.setChecked(True)
        elif msg.data == "navegacion":
            self.radio_button4.setChecked(True)
            self.set_alarma(False)
        elif msg.data == "alarma":
            self.radio_button1.setChecked(False)
            self.radio_button2.setChecked(False)
            self.radio_button3.setChecked(False)
            self.radio_button4.setChecked(False)
            self.set_alarma(True)

    def set_alarma(self, value):
        if self.alarma_preparada:
            if value:
                #self.led_label.setStyleSheet("background-color:red;font : 20px Arial;color:black;")
                self.led_label.setStyleSheet("background-color: rgb(255, 102, 102); font: 20px Arial; color: black;")
                self.alarma = True
                data = rospy.wait_for_message(TOPIC_IMG, Image, timeout=5)
                frame = self.bridge.imgmsg_to_cv2(data, "bgr8") 
                cv2.imwrite("Intruso.jpg", frame) 
                original_pixmap = QPixmap("Intruso.jpg")
                
                # Redimensionar la imagen
                width = 400  # Ancho deseado
                height = 300  # Altura deseada
                resized_pixmap = original_pixmap.scaled(width, height)

                self.image_label.setPixmap(resized_pixmap)
            else:
                self.alarma = False
                self.led_label.setStyleSheet("background-color:black;font : 20px Arial;color:black;")
                self.image_label.clear()

    def on_switch_button_clicked(self):
        # Cambiar el texto del botón según el estado
        if self.switch_button.isChecked():
            self.switch_button.setText('Activado')
            self.alarma_preparada = True    # Transmitir a la máquina de estados que si detecta un intruso debe activar la alarma
            self.pub_alarm.publish(True)
        else:
            self.switch_button.setText('Desactivado')
            self.alarma_preparada = False   # Transmitir a la máquina de estados que debe apagar la alarma y no atenderla
            self.alarma = False
            self.image_label.clear()
            self.led_label.setStyleSheet("background-color:black;font : 20px Arial;color:black;")
            self.pub_alarm.publish(False)


if __name__ == '__main__':

    app = QApplication(sys.argv)
    dark_app = DarkThemeApp()
    sys.exit(app.exec_())
