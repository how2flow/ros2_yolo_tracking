# yolo_tracking

it is ros2-foxy pkg on ODROID-M1.

### Purchase list
- [odroid-M1](https://www.hardkernel.com/shop/odroid-m1-with-4gbyte-ram)
- pan/tilt braket x 1
- sg90 x 2

### requirements
- [ROS2 with NPU](https://wiki.odroid.com/getting_started/os_installation_guide#tab__odroid-m1)
- ros2 (foxy, preinstalled)
- opencv (preinstalled)
- librga-dev (preinstalled)
- odroid-wiringpi
- libwirinpi-dev

### manual
1. Download & Build package.
```
$ git clone https://github.com/how2flow/ros2_yolo_tracking yolo_tracking
$ ln -s ~/ros2_yolo_tracking/motor_srv ~/robot_ws/src
$ ln -s ~/ros2_yolo_tracking ~/robot_ws/src/yolo_tracking
$ cb
```

2. execute nodes.
```
$ ros2 launch yolo_tracking image.launch.py
```
```
$ ros2 run yolo_tracking servo
```
```
$ ros2 run yolo_tracking control
```
