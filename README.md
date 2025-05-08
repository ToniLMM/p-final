# Proyecto final: Robot Ladrón

## Introducción

En esta practica vamos a hacer uso del Kobuki para simular el comportamiento de un robot ladrón, el cuál cuando detecta los objetos marcados del yolo hace sonidos pero si detecta una persona, este huye a la entrada de nuevo.
Este proyecto final hace uso de diferentes aspectos aprendidos en clase como son: la navegación, el sonido del kobuki, el uso de la camara y yolo, el laser...

## Comportamiento básico

## BT

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

![mapa_uni](https://github.com/user-attachments/assets/1decf9cc-d560-4a5b-90ce-31b43fc18364)

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

## Video Final
