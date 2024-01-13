# Robot-Movil-Vigilante
Proyecto final de la asignatura de Robots Móviles, Ingeniería Robótica (2023-2024).

Diseñado para trabajar con un TurtleBot tanto en simulación como con el robot físico en el laboratorio. 

Funcionalidades: 
  - Programación por Gestos del mapeado, la planificación y la generación de trayectorias.
  - Interfaz gráfica para monitoreo y control.
  - Cuenta con un estado de alarma en caso de que durante la navegación se detecte un intruso. En ese caso, toma captura del intruso y lo sigue.
  - Uso de RVIZ para mostrar el estado actualizado del mapa y de los waypoints guardados.


Instrucciones de lanzamiento: 

  SIMULACIÓN
  
    >> roslaunch robot_movil_vigilante museo_navigation.launch                # maquina de estados, simulacion camara, interfaz, Rviz, Stage, Gmapping, amcl
    >> python3.10 Detector_Gestos.py                                          # detección de los gestos, muñeca y cuenta de dedos
    >> python3.10 Detector_Personas.py                                        # detección de personas para detectar intrusos

  ROBOT REAL: turtlebot 
  
    En el terminal del turtlebot: 
        >> roslaunch turtlebot_bringup minimal.launch
        >> roslaunch turtlebot_bringup hokuyo_ust10lx.launch
        >> export TURTLEBOT_3D_SENSOR=astra                                   # en un mismo terminal lanzar ambos comandos
           roslaunch turtlebot_navigation gmapping_demo.launch
        >> roslaunch astra_launch astra.launch                                # importante lanzar la cámara después que el gmapping
    
    En el ordenador: 
        >> roslaunch robot_movil_vigilancia laboratorio_simulation.launch      # máquina de estados, interfaz, Rviz
        >> python3.10 Detector_Gestos.py                                       # detección de los gestos, muñeca y cuenta de dedos
        >> python3.10 Detector_Personas.py                                     # detección de personas para detectar intrusos

  Entrenar el modelo: 
    En ocasiones se ha visto necesario reentrenar el modelo en cada dispositivo a utilizar (actualizar modelo.keras). Para reentrenarlo: 
      >> python3.10 EntrenarModelo.py
