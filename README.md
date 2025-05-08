# Proyecto final: Robot Ladrón

![image](https://github.com/user-attachments/assets/375b8c13-2f68-466f-80bd-9c7d6210fcd3)


## Introducción

En esta practica vamos a hacer uso del Kobuki para simular el comportamiento de un robot ladrón, el cuál cuando detecta los objetos marcados del yolo hace sonidos pero si detecta una persona, este huye a la entrada de nuevo.
Este proyecto final hace uso de diferentes aspectos aprendidos en clase como son: la navegación, el sonido del kobuki, el uso de la camara y yolo, el laser...

## Comportamiento básico

El robot entra en una casa y hace un recorrido de 4 waypoints, la entrada y otros 3 alrededor de ella. En cada waypoint el robot mediante yolo y mientra gira, busca los objetos de valor (botella, ordenador, pelota y mochila), cuando los encuentra emite un sonido, uno diferente por cada objeto. Si en cualquiera de los waypoints detecta a una persona, el robot emite un sonido y seguidamente huye al waypoint de entrada para escapar. Si el robot consigue llegar al waypoint 4 sin ser pillado vuelve a la entrada con todo el botín recolectado

![WhatsApp Image 2025-05-08 at 20 10 21](https://github.com/user-attachments/assets/af478300-70fb-411c-b90d-9c6f9710a410)

## BT

![image](https://github.com/user-attachments/assets/f28fb256-88b9-4562-8178-f75f98cf986c)


## Dependencias

Para usar este programa tendremos que tener instalado los siguientes paquetes:
- [ROS2](https://docs.ros.org/en/jazzy/)
- [Groot](https://github.com/BehaviorTree/Groot): Este es un programa de interfaz gráfica de usuario (GUI) que le permitirá controlar manualmente el robot. Es opcional, pero muy recomendable
- [Yolov8](https://github.com/mgonzs13/yolov8_ros): Es un modelo de visión por computadora que utilizaremos para identificar y filtrar objetos
- [ZeroMQ](https://zeromq.org): Como el árbol de comportamiento es externo a ros, necesita un middleware de comunicación IOT para la comunicación entre nodos, por eso usamos ZMQ.

## Mapeo

```shell
ros2 launch kobuki kobuki.launch.py
ros2 launch slam_toolbox online_sync_launch.py
ros2 run kobuki_keyop kobuki_keyop_node
ros2 run nav2_map_server map_saver_cli -f [nombre_mapa]
```

Mapa inicial:

![image](https://github.com/user-attachments/assets/db9a07fb-374b-4830-af08-14474c9f1ade)

Mapa mejorado:

![image](https://github.com/user-attachments/assets/c82bc5ed-8ab0-495a-8601-5b1fcce9dec6)



## Ejecución

Lanzar kobuki con la cámara y el laser (simulation.launch.py para el simulador)
```shell
ros2 launch kobuki kobuki.launch.py [camara]:=true [laser]:=true
```
Lanzar yolov8
```shell
ros2 launch yolo_bringup yolo.launch.py model:=yolov8m-seg.pt input_image_topic:=/rgb/image_raw
```
Lanzar navigation con el mapa (navigation_sim.launch.py para el simulador)
```shell
ros2 launch kobuki navigation.launch.py map:=$HOME/arquitectura2_ws/src/p-final/thief/maps/mapa_bueno.yaml
```
Lanzar programa
```shell
ros2 launch thief thief.launch.py
```

## Funcionamiento

Para este codigo hemos necesitado 3 nodos: GetWaypoint, Move y Turn

### GetWaypoint

Este nodo, usado en el Behavior Tree, selecciona y devuelve un waypoint en el mapa según una entrada:

  - Si recibe wp_id = "next", devuelve el siguiente waypoint en una lista circular.
  - Si recibe wp_id = "base", devuelve una posición fija llamada "base" (entrada de la casa).

![image](https://github.com/user-attachments/assets/6a2548f4-3ccd-4dc8-a7ef-6f5a75601b37)


### Move

Este nodo, usado en el Behavior Tree, envía una orden de navegación al sistema NavigateToPose de ROS 2:

 1. Lee un objetivo goal del árbol (un PoseStamped).

 2. Envía ese objetivo como acción de navegación.

 3. Informa si la navegación fue exitosa.

![image](https://github.com/user-attachments/assets/f1f9a84c-6a95-428c-ba94-9400e0c9d25a)


### Turn

Este nodo define una acción llamada Turn. Su propósito principal es hacer que el robot gire sobre su eje Z mientras busca ciertos objetos por un sistema de visión basado en YOLO, y responde de forma específica a ellos.

Cuenta con un Callback que guarda las detecciones más recientes recibidas del tópico /yolo/detections.

![imagen](https://github.com/user-attachments/assets/b5f8ee10-0b81-49f0-9271-d1d0dcfcad1e)

Método principal tick():

  - Este método se ejecuta repetidamente como parte del árbol de comportamiento.

  - La deteccion se inicia cuando el estado es IDLE, primero se inicia un temporizador y se limpian las detecciones anteriores.

![imagen](https://github.com/user-attachments/assets/ab8bc9e4-440d-465c-88e6-4c4e3733c05c)

  - Siempre que la variable controladora del tiempo no sobrepase los 8 segundos pasara lo siguiente:

    1. Publica una velocidad angular constante (0.5 rad/s) para hacer que el robot gire.

    2. Detecta objetos y los clasifica segun su ID:

       - Si detecta una persona se detiene, reproduce un sonido, y retorna SUCCESS, y volvera inmediatamente al punto de partida.
         Simulando una huida.

       - Si detecta un objeto de los que le hemos indicado como valioso, hara un sonido indicando que se lo ha llevado.

       - Si no ha detectado ninguna persona durante el barrido de 8 segundos devolvera un FAILURE.

  ![imagen](https://github.com/user-attachments/assets/45ddf09e-5427-4a6d-9aa1-5ac3c4dc8e7a)


## Video Final

https://github.com/user-attachments/assets/ce09696b-f18b-4316-905b-d7908748665f

