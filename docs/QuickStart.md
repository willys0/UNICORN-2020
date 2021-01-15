# Unicorn Quick Start Guide

### Materials
- NVIDIA Jetson TX2 
- RoboRIO
- Pepperl+Fuchs R2000 lidar
- Pepperl+Fuchs R2100 lidar
- Stereolabs ZED stereo camera
- Intel RealSense D435 depth camera
- Usb 3.0 hub
- Husqvarna Automower 430x
- DWM1001 UWB tag


### Connection

Connect via the wi-fi router on the unicorn, network name: "unicorn", password: "unicorn123"

> ssh nvidia@10.0.0.2

password: nvidia

### Introduction

The unicorn has been configured to automatically start the bluetooth driver for playstation 3 controller, as well as roscore and relevant launch files as systemd services. To change what is auto-launched, edit the following file like this:
 
> sudo nano /usr/sbin/roslaunch
 
The sections below assume that nothing is started automatically, disable autolaunch services like:
 
> sudo systemctl disable roscore.service

> sudo systemctl disable roslaunch.service

...and restart.
 
> sudo reboot
 
Re-enable the services with
 
> sudo systemctl enable roscore.service

> sudo systemctl enable roslaunch.service

> sudo reboot

### Software

Make sure that the jetson tx2 is running the latest branch, either using github which requires an internet connection:

> cd ~/catkin_ws/src/<local UNICORN repo> && git fetch

> git remote show origin

> git checkout <correct-branch>

> git pull

You can also set a connected computer as a remote and pull via ssh as demonstrated below, refer to this page for more info: <https://stackoverflow.com/questions/3596260/git-remote-add-with-other-ssh-port>

> cd ~/catkin_ws/src/<local UNICORN repo> 

> git remote add <remote> ssh://<user>@<host>:<port></path/to/repo>/.git

...Or by copying files from your host computer (not recommended):

> cd ~/catkin_ws/src

> scp -r UNICORN nvidia@10.0.0.2:/home/nvidia/catkin_ws/src

Remember to build the code if you update it:

> cd ~/catkin_ws && catkin_make

#### Launch files

The launch files main_2020.launch (to run on the robot) and simulator_2020.launch (for simulating the robot with gazebo) are provided in the unicorn package, some of the most useful arguments are listed below:

- do_slam: (default true)
    - will use SLAM if true
    - if false, a pre-defined map will be loaded and EKF based on wheel encoders and AMCL will be used to localize the robot.
- map_file: (default $(find unicorn_slam)/maps/c2.yaml)
    - decides the map to be loaded when not using SLAM

When SLAMming, the map can be saved using map_saver 

> rosrun map_server map_saver -f map_name

...where map_name is the path where the map will be saved

### Start automower
- Turn on the robot with the switch below the handle.
- Open the hatch and input the pin: `1111`
    - If the automower has ben without power it will ask for a new pin, enter `1111`. I will also want to set up of time and date. Just press ok untill the menu is reached. 
- The loop detection should turn off automaticly if `main_2020.launch` has been launched. If not:
    - Enter settings and hold `7` and `9` until a new options appears.
    - Press the wrench then goto special settings -> override loop detection.
    - Move back to the main menu
    - Press start and close the hatch

For convenience the console interface provided by Husqvarna (hrp_teleop) can be used to see the state of the mower, you want the status to be `PAUSED` to be able to control the mower:

> rosrun am_driver hrp_teleop.py

hrp_teleop can also be used to override loop detection by pressing `9`.

#### Important info
The wireless emergency stop needs to be powered on when you try to use the robot. If it is not powered and within range (it has a large range so line of sight should be sufficient) the robot will not move any of its motors and the state will at best be `STOPPED`.


### Troubleshooting
Here are some common errors and how to fix them:

`FATAL_ERROR`
Shown by hrp_teleop node and indicates that the automower is very sad. Open up the hatch and read the error message to proceed. It usually just want someone to listen to its problems.

`STOPPED`
Shown by hrp_teleop node and indicates that the automower is not started. If you are sure that it has started correctly try to restart the teleop node instead. If you forgot to power the wireless emergency stop, it would also be a good idea to do so now.

`PAUSED`
Shown by hrp_teleop and is actually not an error, it means that everything is good to go!

`SERIAL PORT DOESN'T EXIST`
Shown by am_driver_safe_node started by main_*.launch and indicates that the jetson tx2 cannot find the automower. Make sure that the usb cable is plugged in (hehe) and that ACM is listed under devices.


