# Install docker
curl -sSL https://get.docker.com | sh
sudo usermod -aG docker $USER

# Pull ROS image
docker pull arm64v8/ros:humble

# Build docker image
docker build -t "docker_ros_rover:v1" .

# Docker command to launch ROS2 container for rover
docker run -it --name ros2_rover --net=host --ipc=host --pid=host --device=/dev/serial/by-id/usb-Adafruit_Flora-if00  --device=/dev/serial/by-id/usb-03eb_USB_Roboclaw_2x30A-if00 --device=/dev/input/event0 --device=/dev/input/event2 docker_ros_rover:v1


export DISPLAY=:0

xhost +local:*

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


DISPLAY=:0 ros2 run articubot_one_ui ui_node --ros-args --remap cmd_vel:=diff_cont/cmd_vel_unstamped