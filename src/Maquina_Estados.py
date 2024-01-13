#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

# Librerias para la maquina de estados
import rospy
import smach_ros
from smach import State,StateMachine
from time import sleep
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math

from std_msgs.msg import Int32MultiArray, String, Int32, Bool
from geometry_msgs.msg import Twist, Point, PoseWithCovarianceStamped, PoseArray, Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

# Navegacion a un pto del mapa
import actionlib
from actionlib_msgs.msg import GoalStatus

import warnings
warnings.filterwarnings("ignore")


# ------------------------------------------------------------------------------------------------
# TOPICS Y VARIABLES GLOBALES
# ------------------------------------------------------------------------------------------------

#TOPIC_VEL = "/mobile_base/commands/velocity"   # Turtlebot Real
TOPIC_VEL = "/cmd_vel"      # Turtlebot Simulado 
TOPIC_WRIST = '/wrist_position'
TOPIC_WAYPOINT = '/waypoints'
TOPIC_WAYPOINT_MARKER = '/waypoint_markers'
TOPIC_GESTURES = '/gestos'
TOPIC_FINGERCOUNT = '/num_waypoint'
TOPIC_STATE = '/estados'
TOPIC_DETECTOR = '/persona_detectada'
TOPIC_ALARMA = '/alarma'

# Ángulos en radianes para los rayos 
ANG_IZQ = 30*math.pi/180.0 # a radianes
ANG_DER = -ANG_IZQ

# Cambiar de estado
mapeado = False
planificacion = False
navegacion = False
reposo = True
alarma = False

# Variable para seguir al humano detectado
estado_alarma = False

# Validacion de gestos: repetirse estos gestos para que sea valido
UMBRAL_GESTOS = 2
UMBRAL_ID = 5

# ---------------------------------------------------------------------------

# --- GESTOS
class GestosSubscriber:
    def __init__(self):
        self.ultimo_gesto = None    # ultimo gesto detectado
        self.prev_ultimo_gesto = None   # penultimo gesto detectado 
        self.contador = 0   # contador para saber si el ultimo gesto coincide con el penultimo 
        self.sub = rospy.Subscriber(TOPIC_GESTURES, String, self.gestos_callback)   #topico que publica el gesto detectado en WristDetector

    # Callback: recibe el gesto detectado por WristDetector
    def gestos_callback(self, msg):
        self.prev_ultimo_gesto = self.ultimo_gesto  #actualiza los gestos
        self.ultimo_gesto = msg.data

        if self.ultimo_gesto == self.prev_ultimo_gesto: 
            self.contador += 1  #aumenta el contador si el nuevo gesto es igual al anterior
        else: 
            self.contador = 0

    # Devuelve el ultimo gesto detectado valido (si supera un umbral de veces detectado)
    def get_ultimo_gesto(self):
        global UMBRAL_GESTOS

        if self.contador > UMBRAL_GESTOS:
            # Si se ha reconocido el mismo gesto más de ese umbral de veces: 
            return self.ultimo_gesto
        else: 
            return None
        
# --- CUENTA DEDOS para los ID de los waypoints
class WaypointIdSubscriber:
    def __init__(self):
        self.ultimo_gesto = None    # mismo funcionamiento que para los gestos
        self.prev_ultimo_gesto = None
        self.contador = 0
        self.sub = rospy.Subscriber(TOPIC_FINGERCOUNT, Int32 , self.waypointId_callback) 

    # Callback: recibe los dedos contados por WristDetector
    def waypointId_callback(self, msg):
        self.prev_ultimo_gesto = self.ultimo_gesto  #actualizacion de los gestos
        self.ultimo_gesto = msg.data

        if self.ultimo_gesto == self.prev_ultimo_gesto: 
            self.contador += 1  #aumenta el contador si el ultimo y penultimo gesto son iguales
        else: 
            self.contador = 0

    # Devuelve el numero gestualizado si es valido (si supera un umbral de veces detectado)
    def get_ultimo_gesto(self):
        global UMBRAL_ID

        if self.contador > UMBRAL_ID:
            # Si se ha reconocido el mismo gesto más de ese umbral de veces: 
            return self.ultimo_gesto
        else: 
            return None

# --- POSICION DEL ROBOT
# Obtener las coordenadas actuales del robot pero con Odometria
class RobotOdometry:
    def __init__(self):
        self.current_x = 0.0    #coordenadas x,y (se considera el robot en un plano z=0)
        self.current_y = 0.0
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)

    # Callback: recibe la posicion del robot
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    # Devuelve la posicion del robot
    def get_current_coordinates(self):
        return self.current_x, self.current_y
    
# Hacerlo por amcl
# class RobotPose: 
#     def __init__(self):
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_orientation = None
#         rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)  # Suscribirse al tópico amcl_pose

#     def amcl_pose_callback(self, msg):
#         self.current_x = msg.pose.pose.position.x
#         self.current_y = msg.pose.pose.position.y

#     def get_current_coordinates(self):
#         return self.current_x, self.current_y
        

# --- ESTADOS
class States: 
    def __init__(self):
        self.pub = rospy.Publisher(TOPIC_STATE, String, queue_size=10)

    # Publicar el estado: reposo, mapeado, planificacion, navegacion, alarma.
    # Se usa para la interfaz
    def pub_state(self, msg):
        self.pub.publish(msg)


# --- WAYPOINTS
class Waypoints: 
    def __init__(self):
        self.waypoints_msg = PoseArray()    # lista de waypoints añadidos
        self.waypoint = Pose()  
        self.trayectory = PoseArray()   #trayectoria planeada = lista de waypoints
        self.trayectory_order = []
        self.pub_waypoints = rospy.Publisher(TOPIC_WAYPOINT, PoseArray, queue_size=10)
        self.pub_waypoint_markers = rospy.Publisher(TOPIC_WAYPOINT_MARKER, Marker, queue_size=10)

    # Añadir un waypoint a la lista de waypoints añadidos (estado Mapeado)
    def add_waypoint(self, pose):
        # Crea un waypoint y lo añade a la lista
        self.waypoint = Pose()
        self.waypoint.position.x = pose[0]
        self.waypoint.position.y = pose[1]
        self.waypoints_msg.poses.append(self.waypoint)

        # Crea un marcador en el mapa de Rviz
        self.waypoint_marker = Marker()
        self.waypoint_marker.header.frame_id = "odom"
        self.waypoint_marker.type = Marker.POINTS
        self.waypoint_marker.action = Marker.ADD
        self.waypoint_marker.id = (len(self.waypoints_msg.poses)-1)*2
        text_marker = self.waypoint_marker
        self.waypoint_marker.points.append(Point(pose[0], pose[1], 0.0))

        self.waypoint_marker.scale.x = 0.1
        self.waypoint_marker.scale.y = 0.1
        self.waypoint_marker.color.a = 1.0  # Opacidad
        self.waypoint_marker.color.r = 0.0  # Color rojo
        self.waypoint_marker.color.g = 0.0  # Color verde
        self.waypoint_marker.color.b = 1.0  # Color azul

        self.pub_waypoint_markers.publish(self.waypoint_marker) #publica el waypoint en el mapa
        
        # Añade el texto identificativo al mapa de Rviz
        text_marker.id = text_marker.id+1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.text = f"Waypoint {len(self.waypoints_msg.poses)}"
        text_marker.pose.position.x = pose[0]
        text_marker.pose.position.y = pose[1]
        text_marker.pose.position.z = 0.5
        text_marker.scale.z = 0.2
        text_marker.color.a = 1.0  # Opacidad
        text_marker.color.r = 1.0  # Color rojo
        text_marker.color.g = 1.0  # Color verde
        text_marker.color.b = 1.0  # Color azul
        
        self.pub_waypoint_markers.publish(text_marker)  #publica el texto en el mapa

    # Publicar la lista de waypoints (se recibirá en la interfaz)
    def pub_waypoint_list(self):
        self.pub_waypoints.publish(self.waypoints_msg)

    # Limpiar la lista de waypoints y los marcadores (cada vez que se entre al estado Mapeado)
    def clean_waypoint_list(self):
        self.waypoints_msg = PoseArray()
        self.waypoint_marker = Marker()
        self.waypoint_marker.action = Marker.DELETEALL
        self.pub_waypoint_markers.publish(self.waypoint_marker)

    # Imprimir la lista de waypoints
    def print_waypoint_list(self):
        print(self.waypoints_msg)
        
    # --- TRAYECTORIAS
    # Añadir waypoint a la trayectoria (en el estado Planificacion)
    def add_waypoint_to_trayectory(self, id):
        # Extraer la info de ese waypoint si existe
        if id <= len(self.waypoints_msg.poses): 
            waypoint = self.waypoints_msg.poses[id-1]
            self.trayectory.poses.append(waypoint)
            self.trayectory_order.append(id)
        else: 
            print("WARMING: ", id, " no es un waypoint válido")
    
    # Devuelve la trayectoria
    def get_trayectory(self):
        return self.trayectory

    # Limpia la trayectoria
    def clean_trayectory(self):
        self.trayectory = PoseArray()
        self.trayectory_order = []

    # Imprime la trayectoria
    def print_trayectory(self):
        print(self.trayectory)


# --- ALARMA
class Activacion_Alarma: 
    def __init__(self):
        self.alarma_subscriber = rospy.Subscriber(TOPIC_ALARMA, Bool , self.alarma_callback) 

    # Callback: si se activa o desactiva la alarma por la interfaz se modifica esta variable global
    def alarma_callback(self, msg):
        global estado_alarma
        estado_alarma = msg.data

# --------------------------------------------------------------------------------
# -------- ESTADOS DE LA MAQUINA DE ESTADOS --------------------------------------

# --- ESTADO REPOSO: mantenerse en este estado con robot parado hasta detectar el gesto de inicio de otro estado -------------------------- #
class Reposo(State):

    def __init__(self, gestos_subscriber):     # Inicializacion
        State.__init__(self, outcomes=['mapeado', 'planificacion', 'navegacion']) 
        self.gestos_subscriber = gestos_subscriber

        self.estados_pub = States()

    # Ejecucion del estado
    def execute(self, userdata):
        global mapeado, planificacion, navegacion, reposo

        rate = rospy.Rate(10)

        # Mostrar por terminal y por la interfaz el estado actual
        print(':: Estado - Reposo')
        self.estados_pub.pub_state("reposo")    

        while reposo:

            ultimo_gesto = self.gestos_subscriber.get_ultimo_gesto()    #obtener el ultimo gesto
            if ultimo_gesto is not None:    #si el gesto es valido (reconocido mas veces que un umbral)
                
                # Estado Mapeado
                if ultimo_gesto == "seguir":
                    #print("Detectado gesto Mapeado")
                    mapeado = True
                    reposo = False

                # Estado Planifiacion
                if ultimo_gesto == "waypoint":
                    #print("Detectado gesto Planificacion")
                    planificacion = True
                    reposo = False
                
                # Estado Navegacion
                if ultimo_gesto == "okay":
                    #print("Detectado gesto Navegacion")
                    navegacion = True
                    reposo = False

            rate.sleep()
        
        if mapeado:
            return 'mapeado'
        if planificacion:
            return 'planificacion'
        if navegacion:
            return 'navegacion'


# --- ESTADO MAPEADO: seguir al humano y guardar waypoints ------------------------------------------------- #
class Mapeado(State):

    def __init__(self, gestos_subscriber, waypoints_subscriber):     # Inicializacion
        State.__init__(self, outcomes=['end_mapeado']) 
        # Publishers
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5) # publicar la velocidad
        self.estados_pub = States()

        # Subscriptores y demas
        self.gestos_subscriber = gestos_subscriber
        self.waypoints = waypoints_subscriber
        self.robot_coordinates = RobotOdometry()    #utilizar la odometria para guardar la pose del robot
        #self.robot_coordinates = RobotPose()   #utilizar amcl
    
    # Ejecucion del estado
    def execute(self, userdata):
        global mapeado, reposo 
        mapeado = True
        prev_waypoint = False   # se emplea para que solo se añada waypoint nuevo la primera vez que se detecta

        # Mostrar por terminal y por la interfaz el estado actual
        print(':: Estado - Mapeado')
        self.estados_pub.pub_state("mapeado")

        # Suscribers
        self.subWrist = rospy.Subscriber(TOPIC_WRIST, Int32MultiArray , self.wrist_position_callback) 
        rate = rospy.Rate(10)

        self.waypoints.clean_waypoint_list()    # limpia la lista de waypoints previa
        self.waypoints.pub_waypoint_list()  # actualizar la lista en en la interfaz

        while mapeado:

            ultimo_gesto = self.gestos_subscriber.get_ultimo_gesto()    #obtener el ultimo gesto
            if ultimo_gesto is not None:    #si el gesto es valido (reconocido mas veces que un umbral)
                
                # Gesto de Cancelar estado
                if ultimo_gesto == "cancel":
                    # Vuelta al estado "REPOSO"
                    reposo = True
                    mapeado = False
                    prev_waypoint = False

                # Gesto de Seguir (palma abierta)
                elif ultimo_gesto == "stop":
                    # El robot sigue al humano con ese gesto (se hace en el callback)
                    #print("Seguir")
                    prev_waypoint = False

                # Gesto de Añadir Waypoint: solo se crea waypoint si es la primera vez que se detecta (sino se publicarian hasta que deje de detectarse)
                elif ultimo_gesto == "waypoint" and prev_waypoint == False:
                    # Añadir un waypoint + publicarlo
                    print("Añadir waypoint")
                    coordenadas_actuales = self.robot_coordinates.get_current_coordinates()  
                    self.waypoints.add_waypoint(coordenadas_actuales)
                    self.waypoints.pub_waypoint_list()
                    self.waypoints.print_waypoint_list()

                    prev_waypoint = True   # asegurar que solo se entra la primera vez

                # Gesto de Parar (puño)
                elif ultimo_gesto == "stop":
                    # Parar el robot
                    #print("Parar robot")
                    cmd = Twist()
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.pub.publish(cmd)
                    prev_waypoint = False

            rate.sleep()

        # Limpieza de los suscriptores:
        if not mapeado:
            self.subWrist.unregister()                               
       
        return 'end_mapeado'    #pasa a reposo
    
    # Seguir muñeca: callback cuando se detecta mano (muñeca)
    def wrist_position_callback(self, msg):

        ultimo_gesto = self.gestos_subscriber.get_ultimo_gesto()

        # si el gesto es seguir muñeca y NO es parar: sigue al humano
        if ultimo_gesto is not None and ultimo_gesto == "stop":

            wrist_diff = msg.data[0]    # diferencia de posición respecto al centro
            area = msg.data[1]          # área del bounding box

            # ----- Cambiar Vx con Area 
            if area > 110000 or area < 2500:
                vx = 0.0
            else:
                # Mayor área, más lento, menor área, más rápido. En 2500 debe valer 0.5, en 111000 debe valer 0
                vx = 1250/area

            # ----- Cambiar Vz con diff
            # Positivo, girar a la derecha, negativo, girar a la izquierda
            # El ancho de la imagen es 600, como mucho diff valdrá {300, -300}, en ese caso debe valer 1
            vz = (-1)*wrist_diff / 250

            cmd = Twist()
            cmd.linear.x = vx
            cmd.angular.z = vz

            self.pub.publish(cmd)   #publica el comando de velocidad


# --- ESTADO PLANIFICACION: generar la trayectoria de waypoints -------------------------- #
class Planificacion(State):
        
    def __init__(self, gestos_subscriber, waypoints_subscriber):     # Inicializacion
        State.__init__(self, outcomes=['end_planificacion', 'stop']) 
        self.estados_pub = States()
        
        self.gestos_subscriber = gestos_subscriber
        self.waypoints = waypoints_subscriber
        self.waypoints_subscriber = WaypointIdSubscriber()

    # Ejecucion del estado
    def execute(self, userdata):
        rate = rospy.Rate(10)
        global planificacion, reposo, navegacion
        prev_ultimo_id = "2"    #ya que por defecto entras con dos dedos al estado

        # Mostrar por terminal y por la interfaz el estado actual
        self.estados_pub.pub_state("planificacion")

        rate = rospy.Rate(10)

        self.waypoints.clean_trayectory()   # limpiar la trayectoria
        
        while planificacion: 

            # mirar si se ha pedido parar o si se ha confirmado la trayectoria
            ultimo_gesto = self.gestos_subscriber.get_ultimo_gesto()
            if ultimo_gesto is not None:    #si el gesto es valido (reconocido mas veces que un umbral)
                
                #Gesto de Cancelar
                if ultimo_gesto == "cancel":
                    # Vuelta al estado "REPOSO"
                    reposo = True
                    planificacion = False

                #Gesto de Confirmar la trayectoria
                if ultimo_gesto == "okay":
                    # Acabada la trayectoria: ir directamente a Navegacion
                    navegacion = True
                    planificacion = False

                # Añadir un waypoint a la trayectoria
                ultimo_id = self.waypoints_subscriber.get_ultimo_gesto()
                if ultimo_id is not None and ultimo_id > 0: 
                    
                    if ultimo_id != prev_ultimo_id:
                        # nuevo waypoint
                        print("Waypoint añadido: ", ultimo_id)
                        self.waypoints.add_waypoint_to_trayectory(ultimo_id)
                        self.waypoints.print_trayectory()

                    prev_ultimo_id = ultimo_id  # asegurar que solo se añade 1 vez

                rate.sleep()
            
        if( reposo):
            print("Parado el estado Planificacion")
            return 'stop'
        if( navegacion ):
            print("Trayectoria confirmada, entrando al estado Navegación")
            return 'end_planificacion'


# --- ESTADO NAVEGACIÓN: moverse a los waypoints seleccionados en el estado anterior -------------------------- #
class Navegacion(State):
        
    def __init__(self, gestos_subscriber, waypoints_subscriber):     # Inicializacion
        State.__init__(self, outcomes=['end_navigation', 'stop', 'alarma', 'None']) 

        self.estados_pub = States()
        self.gestos_subscriber = gestos_subscriber
        self.waypoints = waypoints_subscriber

        # Publishers
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5) # publicar la velocidad

        self.interrumpir_navegacion = False

    # Ejecucion del estado
    def execute(self, userdata):
        rate = rospy.Rate(10)
        global navegacion, reposo, estado_alarma, alarma
        self.interrumpir_navegacion = False

        # Mostrar por terminal y por la interfaz el estado actual
        self.estados_pub.pub_state("navegacion")

        self.client =  actionlib.SimpleActionClient('move_base',MoveBaseAction) #cliente ROS
        self.client.wait_for_server() #esperar que el nodo 'move_base' este activo

        # Suscribers
        self.subDetector = rospy.Subscriber(TOPIC_DETECTOR, Int32MultiArray , self.human_position_callback) 

        # Obtener la trayectoria definida en planificacion
        trayectory = self.waypoints.get_trayectory()
        if len(trayectory.poses) > 0: 

            # Mientras que no se haya pedido parar o se entre en el estado alarma
            while navegacion: 

                # Hacer la trayecoria definida
                for i, waypoint_pose in enumerate(trayectory.poses):
                    # Recorrer todos los waypoints
                    print("Destino: Waypoint ", self.waypoints.trayectory_order[i])
                    self.moveTo(waypoint_pose.position.x, waypoint_pose.position.y)
                    # en moveTo se va al punto y se comprueba que no se detecta el gesto de cancelar

                # Se ha terminado la trayectoria: Comportamiento normal 
                # (descomentar si solo quiere hacerse la trayectoria 1 vez y no en ciclo)
                # if( not estado_alarma ):
                    # print("Finalizada la trayectoria con éxito")
                    # reposo = True
                    # navegacion = False

                # Si se ha detectado intruso y hay estado de alarma
                if( estado_alarma and self.interrumpir_navegacion ):
                    print("Finalizada la Navegación: Detección de Intruso")
                    reposo = False
                    navegacion = False
                    alarma = True
        else:
            print("No hay trayectoria")
            reposo = True
            navegacion = False

            
        if( reposo ):
            print("Parado el estado Navegacion")
            return 'end_navigation'
        if( alarma ):
            print("Parado el estado Navegación: Entrando en Alarma")
            return 'alarma'
        return 'end_navigation'

    # Ir al waypoint de la trayectoria
    def moveTo(self, x, y):
        global navegacion, reposo

        #un MoveBaseGoal es un punto objetivo al que nos queremos mover
        goal = MoveBaseGoal()
        #sistema de referencia que estamos usando
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.pose.position.x = x   
        goal.target_pose.pose.position.y = y
        #La orientación es un quaternion. Tenemos que fijar alguno de sus componentes
        goal.target_pose.pose.orientation.w = 1.0

        #enviamos el goal 
        self.client.send_goal(goal)
        #vamos a comprobar cada cierto tiempo si se ha cumplido el goal: get_state obtiene el resultado de la acción 
        state = self.client.get_state()
        
        #ACTIVE es que está en ejecución, PENDING que todavía no ha empezado
        while state==GoalStatus.ACTIVE or state==GoalStatus.PENDING:
            rate = rospy.Rate(10)   #esto nos da la oportunidad de escuchar mensajes de ROS
            state = self.client.get_state()

            # Interrumpir trayectoria (si se ha detectado instruso)
            if self.interrumpir_navegacion: 
                self.client.cancel_goal()   # cancelar la meta actual

            # Mirar si se ha pedido parar por gesto
            ultimo_gesto = self.gestos_subscriber.get_ultimo_gesto()
            if ultimo_gesto is not None:    
                
                if ultimo_gesto == "cancel":
                    # Vuelta al estado "REPOSO"
                    print("Cancelado Navegacion")
                    self.client.cancel_goal()  # Cancelar la meta actual
                    reposo = True
                    navegacion = False
                    self.interrumpir_navegacion = True
                    break

            rate.sleep()

        return self.client.get_result()
    
    # Intruso detectado
    def human_position_callback(self, msg):
        global estado_alarma

        if( estado_alarma ):
            self.interrumpir_navegacion = True  # parar la navegacion a los waypoints
        else: 
            self.interrumpir_navegacion = False
            

# --- ESTADO ALARMA: seguir al instruso -------------------------- #
class Alarma(State):
        
    def __init__(self, gestos_subscriber):     # Inicializacion
        State.__init__(self, outcomes=['stop', 'navegacion']) 

        self.estados_pub = States()
        self.gestos_subscriber = gestos_subscriber

        # Publishers
        self.pub = rospy.Publisher(TOPIC_VEL, Twist, queue_size=5) # publicar la velocidad

        self.humano_detectado = False
        self.timer = rospy.Time.now()

    # Ejecucion del estado
    def execute(self, userdata):
        rate = rospy.Rate(10)
        global alarma, reposo, navegacion, estado_alarma
        self.interrumpir_navegacion = False

        # Mostrar por terminal y por la interfaz el estado actual
        self.estados_pub.pub_state("alarma")

        # Suscribers
        self.subDetector = rospy.Subscriber(TOPIC_DETECTOR, Int32MultiArray , self.human_position_callback) 

        while( alarma ):
            
            # Se ha salido del estado alarma por la interfaz
            if( not estado_alarma ):
                alarma = False
                reposo = True

            # Se ha perdido al intruso por más de x tiempo
            last_detected_time = rospy.Time.now() - self.timer    # tiempo desde que se detecto

            if last_detected_time.to_sec() > 5.0: 
                alarma = False
                navegacion = True


        if( reposo):
            print("Parado el estado Alarma manualmente")
            return 'stop'
        
        if( navegacion ):
            print("Intruso Perdido: Pasando a modo Navegacion")
            return 'navegacion'
        
    
    # Seguir humano
    def human_position_callback(self, msg):

        global estado_alarma

        if( estado_alarma ):

            self.timer = rospy.Time.now()   # reinicia cada vez que se detecta

            human_diff = msg.data[0]    # diferencia de posición respecto al centro

            vx = 0.5

            # ----- Cambiar Vz con diff
            # Positivo, girar a la derecha, negativo, girar a la izquierda
            # El ancho de la imagen es 600, como mucho diff valdrá {300, -300}, en ese caso debe valer 1
            vz = (-1)*human_diff / 250

            cmd = Twist()
            cmd.linear.x = vx
            cmd.angular.z = vz

            self.pub.publish(cmd)   #publicar la velocidad
    

# --- MAIN --------------------------------------------------------------------------- #
if __name__ == '__main__':

    # Inicio del nodo de ROS y de la maquina de estados (sm)
    rospy.init_node("maquina_estados")

    # Subscriptores compartidos: gestos, waypoints y alarma
    gestos_subscriber = GestosSubscriber()
    waypoints_subscriber = Waypoints()
    alarma_subscriber = Activacion_Alarma()

    sm = StateMachine(outcomes=['stop'])

    # Definicion de la maquina de estados
    with sm:
        
        # Estado REPOSO: robot parado esperando al gesto para cambiar de estado
        StateMachine.add('Reposo', Reposo(gestos_subscriber), transitions={'mapeado':'Mapeado', 'planificacion':'Planificacion', 'navegacion':'Navegacion'})

        # Estado MAPEADO: inicar el mapeado del robot siguiendo al humano y guardando waypoints
        StateMachine.add('Mapeado', Mapeado(gestos_subscriber, waypoints_subscriber), transitions={'end_mapeado':'Reposo'})

        # Estado PLANIFICACION: marcar por gestos los waypoints que forman la trayectoria del estado 'Navegacion'
        StateMachine.add('Planificacion', Planificacion(gestos_subscriber, waypoints_subscriber), transitions={'end_planificacion':'Navegacion', 'stop':'Reposo'})

        # Estado NAVEGACION: hacer la navegacion de la trayectoria del estado 'Planificacion'
        StateMachine.add('Navegacion', Navegacion(gestos_subscriber, waypoints_subscriber), transitions={'end_navigation':'Reposo', 'stop':'Reposo', 'alarma':'Alarma', 'None':'Reposo'})

        # Estado ALARMA: seguir al intruso si se ha establecido el estado de alarma
        StateMachine.add('Alarma', Alarma(gestos_subscriber), transitions={'stop':'Reposo', 'navegacion':'Navegacion'})


    # Ejecutar la máquina de estados
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()