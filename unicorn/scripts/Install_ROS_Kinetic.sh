#!/bin/bash

# Script for installing ROS KINETIC.

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# Update libraries
sudo apt-get update -y

# Install ROS
sudo apt-get install ros-kinetic-desktop-desktop -y

# Install packages needed for 
sudo apt-get install ros-kinetic-effort-controllers -y
sudo apt-get install ros-kinetic-gmapping -y
sudo apt-get install ros-kinetic-gazebo-ros -y
sudo apt-get install ros-kinetic-navigation -y
sudo apt-get install ros-kinetic-range-sensor-layer -y
sudo apt-get install ros-kinetic-global-planner -y
sudo apt-get install ros-kinetic-teb-local-planner -y

sudo rosdep init
rosdep update

# Edit in .bashrc to find the ros installation when opening a terminal.
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y

