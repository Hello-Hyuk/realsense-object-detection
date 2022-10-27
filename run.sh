xhost + && \
docker run -it \
-v ~/Dev/realsense-object-detection/wego_ws:/workspace/wego_ws \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=unix$DISPLAY \
--net=host \
--rm \
--privileged \
--name test_ros \
--gpus all \
test_ros
