# UNICORN

The trash-collecting robot.

This project was made using the ROS navigation stack.

* [ROS Navigation](http://wiki.ros.org/navigation/Tutorials/RobotSetup) - ROS Navigation stack setup guide

## Getting Started

Please read through the coding style guides we are using

* [CppStyleGuide](http://wiki.ros.org/CppStyleGuide) - ROS Cpp Style Guide
* [PyStyleGuide](http://wiki.ros.org/PyStyleGuide) - ROS Python Style Guide

## Dependencies

```
sudo apt-get install ros-kinetic-effort-controllers
sudo apt-get install ros-kinetic-gmapping
sudo apt-get install ros-kinetic-gazebo-ros
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-range-sensor-layer
sudo apt-get install ros-kinetic-global-planner
sudo apt-get install ros-kinetic-teb-local-planner
cd ~/catkin_ws/src
git clone https://github.com/HusqvarnaResearch/hrp.git
```

Goto hrp/am_driver_safe and insert a CATKIN_IGNORE file.
```
cd hrp/am_driver_safe
touch CATKIN_IGNORE
```

Then continue getting other packages.
```
cd ~/catkin_ws/src
git clone https://github.com/clearpathrobotics/LMS1xx.git
git clone https://github.com/husky/husky.git
cd ~/catkin_ws/src/husky
git checkout kinetic-devel
cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -r -y
cd ~/catkin_ws && catkin_make
```

## Documentation

Generate documentation using rosdoc_lite by running:
> ./generate_doc.sh

Then access the documentation by running:
> xdg-open docs/doc/html/annotated.html

Or by going to the html folder and double-clicking on annotated.html.

Please refer to docs/QuickStart.md for help on how to launch the platform.

## How to Git

Please refer to this cheat sheet before doing anything.

* [GitCheatSheet](https://services.github.com/on-demand/downloads/github-git-cheat-sheet.pdf) - Git Cheat Sheet

Open a command window and run:

```
man git
```

To access the git manual.

#### Clone the repository

```
cd ~/catkin_ws/src
git clone https://github.com/notlochness/UNICORN.git
```

#### Checkout the branch you want and make a local one

```
git branch --list
git checkout <branch>
git branch <new-branch>
```

#### When your code is stable merge the branches

```
git checkout <branch>
git merge <local-branch>
```

If you are working on the same branch it's good practice to always run a fetch & pull when you start your workday.

```
git fetch
git pull
```

#### Push your changes remotely with a commit message

```
git add .
git commit -m "Commit message goes here"
git push origin <branch> 
```

