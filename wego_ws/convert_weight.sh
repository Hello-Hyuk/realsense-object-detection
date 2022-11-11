#!/bin/bash
source devel/setup.bash
cd /workspace/wego_ws/src/tensorrt_demos/plugins 
make
cd /workspace/wego_ws
catkin_make && source devel/setup.bash
# cd src/tensorrt_demos/yolo/
# python3 yolo_to_onnx.py -m red_hand
# python3 onnx_to_tensorrt.py -v -m red_hand
