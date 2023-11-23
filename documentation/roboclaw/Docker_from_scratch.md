# Install docker
curl -sSL https://get.docker.com | sh
sudo usermod -aG docker $USER

# Pull ROS image
docker pull arm64v8/ros:humble