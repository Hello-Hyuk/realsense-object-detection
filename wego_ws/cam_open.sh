#!/bin/bash
source devel/setup.bash
catkin_make && source devel/setup.bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true