# Install docker
curl -sSL https://get.docker.com | sh
sudo usermod -aG docker $USER

# Pull ROS image
docker pull arm64v8/ros:humble






# Build docker image
docker build -t "docker_ros_rover:v1" .