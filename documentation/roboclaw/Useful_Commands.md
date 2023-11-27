# Launch Gazebo Sim with saved workd
ros2 launch rover launch_sim.launch.py world:=src/rover/worlds/obstacles.world 

# Launch teleop Keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard 
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped

# Launch roboclaw hardware interface
ros2 launch diffdrive_roboclaw diffbot.launch.py use_mock_hardware:=false gui:=false

# Real Robot teleop
ros2 launch rover launch_rover_bot.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped


# Docker command to launch ROS2 container for rover
docker run -it --name ros2_rover --net=host --ipc=host --pid=host --device=/dev/serial/by-id/usb-Adafruit_Flora-if00  --device=/dev/serial/by-id/usb-03eb_USB_Roboclaw_2x30A-if00 --device=/dev/input/event0 --device=/dev/input/event2 docker_ros_rover:v1


xhost +local:*

docker run -it \
--name ros2_rover \
--device=/dev/dri \
--volume=/tmp/.X11-unix:/tmp/.X11-unix \
--env="DISPLAY=$DISPLAY" \
--net=host \
--ipc=host \
--pid=host \
--device=/dev/serial/by-id/usb-Adafruit_Flora-if00  \
--device=/dev/serial/by-id/usb-03eb_USB_Roboclaw_2x30A-if00 \
--device=/dev/input/event0 \
--device=/dev/input/event2 \
docker_ros_rover:v1