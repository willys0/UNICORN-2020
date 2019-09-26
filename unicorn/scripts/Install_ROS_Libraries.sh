#!/bin/bash

# Script for Downloading and making essential code for the UNICORN project

# Download the code from Husqvarna to run the mower and make changes for the project that are needed.
cd ~/catkin_ws/src
git clone https://github.com/HusqvarnaResearch/hrp.git
cd hrp/am_driver_safe
touch CATKIN_IGNORE


# For path planning
cd ~/catkin_ws/src
git clone https://github.com/husky/husky.git
cd ~/catkin_ws/src/husky
git checkout kinetic-devel


# Check and install all dependencies
cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -r -y
cd ~/catkin_ws && catkin_make

