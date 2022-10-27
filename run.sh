xhost + && \
docker run -it \
-v ~/docker_ws/test_ws/wego_ws:/workspace/wego_ws \
-v /tmp/.X11-unix:/tmp/.X11-unix:ro \
-e DISPLAY=unix$DISPLAY \
--net=host \
--rm \
--privileged \
--name test_ros \
--gpus all \
test_ros
