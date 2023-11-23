# Python Base Image from https://hub.docker.com/r/arm32v7/python/
FROM arm64v8/ros:humble

# Clone the rover project repo from github
RUN git clone https://github.com/muditguptacode/rover-v2.git

# Install the ros packages
RUN apt update && apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-dev-tools libboost-all-dev -y 

# Update dependencies
# WORKDIR /rover-v2/colcon_ws
# RUN rosdep update
# RUN rosdep install --from-paths src --ignore-src -r -y

# Source the workspace
WORKDIR /rover-v2/colcon_ws
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install
RUN . install/setup.sh
