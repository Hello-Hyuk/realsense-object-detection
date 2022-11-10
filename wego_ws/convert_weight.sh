cd src/tensorrt_demos/yolo/
python3 yolo_to_onnx.py -m red_hand
python3 onnx_to_tensorrt.py -v -m red_hand
