export DISPLAY=:0

xhost +local:*

docker rm -f ros2_rover

docker run \
-it \
--detach \
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

# Wait for docker to start
sleep 5

# Launch Main rover package
docker exec -d ros2_rover bash -c "source /opt/ros/humble/setup.bash && source /rover-v2/colcon_ws/install/setup.bash && ros2 launch rover launch_rover_bot_joystick.launch.py"

# Launch http server and webui
docker exec -w "/rover-v2/webui/" -d ros2_rover bash -c "http-server ."
sleep 5
DISPLAY=:0 chromium-browser -kiosk --incognito http://10.10.20.152:8080