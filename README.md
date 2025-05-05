# Proyecto final: Robot Ladrón

## Introducción

## Comportamiento básico

## BT

## Dependencias

## Mapeo

```shell
ros2 launch kobuki kobuki.launch.py
ros2 launch slam_toolbox online_sync_launch.py
ros2 run kobuki_keyop kobuki_keyop_node
ros2 run nav2_map_server map_saver_cli -f [nombre_mapa]
```

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
