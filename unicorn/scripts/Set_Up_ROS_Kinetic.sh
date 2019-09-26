#!/bin/bash

# Script for setup the ROS Kinetic workspace

# Create the workspace and make it
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make

# Add the path to the devel map of the ROS kinetic workspace
source ~/catkin_ws/devel/setup.bash
