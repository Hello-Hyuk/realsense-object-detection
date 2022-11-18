# realsense-object-detection

## Prerequisite
Object
- Red hand stick

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
자동으로 아래의 3개의 터미널이 생성되며 sh파일이 실행됨
```
$ ./run.sh
```

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
$ rosrun rviz rviz -d ~/wego_ws/src/rviz/red_hand.rviz 
```
## Exit
```
$ ./stop.sh
```



