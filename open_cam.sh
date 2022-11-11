xhost + && \
docker run -it \
-v ~/realsense-object-detection/wego_ws:/workspace/wego_ws \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=unix$DISPLAY \
--net=host \
--rm \
--privileged \
--name wego_detection \
--gpus all \
wego_realsense \
/workspace/wego_ws/shell/cam_open.sh
