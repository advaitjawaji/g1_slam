#!/bin/bash
set -e

echo "=== Installing ROS2 Humble on Ubuntu 22.04 ==="

# Locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ROS2 apt repo
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-rtabmap-ros \
    ros-humble-realsense2-camera \
    ros-humble-nav2-bringup \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete

# rosdep
sudo rosdep init || true
rosdep update

# Source ROS2 automatically
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

echo ""
echo "=== Done! Run: source ~/.bashrc ==="
echo "Then build the workspace with:"
echo "  cd ~/Desktop/g1_slam/ros2_ws && colcon build"
