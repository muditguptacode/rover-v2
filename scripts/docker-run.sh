export DISPLAY=:0

xhost +local:*

docker rm -f ros2_rover

docker run -it \
--name ros2_rover \
--device=/dev/dri \
--privileged \
--volume=/tmp/.X11-unix:/tmp/.X11-unix \
--env="DISPLAY=$DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--net=host \
--ipc=host \
--pid=host \
-v /dev/input:/dev/input \
-v /dev/bus/usb:/dev/bus/usb \
-v /dev/serial:/dev/serial \
docker_ros_rover:v1