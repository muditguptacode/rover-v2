# Python Base Image from https://hub.docker.com/r/arm32v7/python/
FROM arm64v8/ros:humble

# Clone the rover project repo from github
run git clone https://github.com/muditguptacode/rover-v2.git

# Install the ros packages
RUN apt install ros-humble-ros2-control ros-humble-ros2-controllers

# Update dependencies
WORKDIR /rover-v2/colcon_ws
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

# Source the workspace
WORKDIR /rover-v2/colcon_ws
RUN source /opt/ros/humble/setup.sh 
RUN colcon build --symlink-install --executor sequential
RUN source install/setup.sh
