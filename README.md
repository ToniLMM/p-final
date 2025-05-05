# Proyecto final: Robot Ladrón

## Introducción

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
ros2 launch yolov_bringup yolov8.launch.py input_image_topic:=/camera/rgb/image_raw image_reliability:=0 model:=yolov8m-seg.pt
```
Lanzar navigation con el mapa (navigation_sim.launch.py para el simulador)
```shell
ros2 launch kobuki navigation.launch.py map:=/home/tonilmm/arquitectura2_ws/src/p-final/pack1/maps/lab_map_2.yaml 
```
Lanzar programa
```shell
ros2 launch waiter_cpp waiter.launch.py
```

## Funcionamiento

## Video Final
