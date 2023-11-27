export DISPLAY=:0

xhost +local:*

docker rm -f ros2_rover

docker run -it \
--name ros2_rover \
--device=/dev/dri \
--volume=/tmp/.X11-unix:/tmp/.X11-unix \
--env="DISPLAY=$DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--net=host \
--ipc=host \
--pid=host \
--device=/dev/serial/by-id/usb-Adafruit_Flora-if00  \
--device=/dev/serial/by-id/usb-03eb_USB_Roboclaw_2x30A-if00 \
--device=/dev/input/by-id/usb-Valve_Software_Steam_Controller-event-mouse \
docker_ros_rover:v1