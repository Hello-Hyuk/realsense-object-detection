# realsense-object-detection

## Prerequisite
H/W 
- realsense camera 

S/W
- nvidia-driver
- docker
- nvidia-docker
- ROS (if you want to communicate with docker containor)

## clone repository
**clone this repo in home directory**
```
cd ~/
git clone https://github.com/Hello-Hyuk/realsense-object-detection.git
```

## build dockerfile 
```
$ cd realsense-objeect-detection/docker
$ ./docker_build.sh
```
시간이 어느정도 소요됨

## init docker containor
**Just do it once**
```
$ ./init_run.sh
```

## Start
open 3 terminal in local in realsense-object-detection direcory

**first terminal (open realsense camera)** 
```
$ ./open_cam.sh
```

**second terminal (Yolo detection)**
```
$ ./object_detect.sh
```

**third terminal (publish object 3d coordinate)**
```
$ ./get_object_position.sh
```
## display

open terminal and execute rviz 
```
$ rviz
```
open rviz config file
```
file -> open config -> realsense/wego_ws/src/rviz/red_hand.rviz
```
## Exit

ctrl+c on first terminal




