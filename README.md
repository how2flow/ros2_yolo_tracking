# yolo_tracking

it is ros2-foxy pkg on ODROID-M1.

### Purchase list
- [odroid-M1](https://www.hardkernel.com/shop/odroid-m1-with-4gbyte-ram)
- pan/tilt braket x 1
- sg90 x 2

### requirements
- [ubuntu-ros2-odroidm1](https://dn.odroid.com/RK3568/ODROID-M1/Ubuntu/ubuntu-20.04-ros2-odroidm1-20230104.img.xz)
- ros2-foxy
- opencv
- librga-dev

### manual
1. Download & Build package.
```
$ git clone https://github.com/how2flow/ros2_yolo_tracking yolo_tracking
$ ln -s ~/yolo_tracking ~/robot_ws/src
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
