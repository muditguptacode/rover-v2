# Python Base Image from https://hub.docker.com/r/arm32v7/python/
FROM arm64v8/ros:humble


# Install the ros packages
RUN apt update && apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-dev-tools ros-humble-xacro ros-humble-joy ros-humble-teleop-twist-joy libboost-all-dev -y 
RUN apt update && apt upgrade -y
RUN apt install python3-pip python3-tk ros-humble-rosbridge-server nano iputils-ping evtest ros-humble-teleop-twist-keyboard nodejs npm -y
RUN npm install http-server -g

# Copy the rover project repo from github
COPY ./rover-v2 /rover-v2 

# Start web server
# WORKDIR /rover-v2/webui
# CMD http-server -p 8080 &

# Source the workspace
WORKDIR /rover-v2/colcon_ws
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install
RUN . install/setup.sh

# Set .bashrc to source the workspace
WORKDIR /root
RUN touch .bashrc
RUN echo "source /rover-v2/colcon_ws/install/setup.bash" >> .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> .bashrc

# GO back to main directory
WORKDIR /rover-v2/colcon_ws