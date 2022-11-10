# realsense-object-detection
## clone repository
**clone this repo in home directory**
```
cd ~/
git clone https://github.com/Hello-Hyuk/realsense-object-detection.git
```

## build dockerfile 
```
$ cd realsense-objeect-detection
$ ./docker_build.sh
```
시간이 어느정도 소요됨


## init docker containor
**Just do once when you clone this repo**

```
$ ./run.sh
```

### convert yolo weight to tensorrt
```
# ./convert_weight.sh
# cd /workspace/wego_ws
# catkin_make
# source devel/setup.bash
```

## Start
open 3 terminal in local

**first terminal (open realsense camera)**
```
$ ./run.sh
# ./cam_open.sh
```

**second termianl (Yolo detection)**
```
$ ./run_terminal.sh
# ./object_detect.sh
```

**third terminal (publish object 3d coordinate)**
```
$ ./run_terminal.sh
# ./get_object_position.sh
```








