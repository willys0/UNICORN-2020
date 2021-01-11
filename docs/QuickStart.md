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

> sudo systemctl disable sixad.service
 
...and restart.
 
> sudo reboot
 
Re-enable the services with
 
> sudo systemctl enable roscore.service

> sudo systemctl enable roslaunch.service

> sudo systemctl enable sixad.service

> sudo reboot

#### Important info
The wireless emergency stop needs to be powered on when you try to use the robot. If it is not powered and within range (it has a large range so line of sight should be sufficient) the robot will not move any of its motors.


### Software

Make sure that the jetson tx2 is running the latest branch, either using github which requires an internet connection:

> roscd unicorn && git fetch

> git remote show origin

> git checkout <correct-branch>

> git pull

Or by copying files from your host computer:

> cd ~/catkin_ws/src

> scp -r UNICORN nvidia@10.42.0.1:/home/nvidia/catkin_ws/src

Copy this function into ~/.bashrc if you will be updating the platform frequently.

```
scp_file()
{
    my_pwd=$(pwd)
    new_pwd=/home/nvidia/${my_pwd:15}/
    scp -r "$@" nvidia@10.42.0.1:"$new_pwd"
}
```

Usage example:

> scp_file UNICORN/

Remember to build the code if you update it:

> cd ~/catkin_ws && catkin_make

#### Launch File

The launch files main_*.launch all have the same arguments available. 

- use_gmapping: true if a map should be generated from laser scan.
    + The map created using gmapping may be saved using the map_server:

> rosrun map_server map_saver -f map_name

- map_file: path/to/map.yaml to use if `use_gmapping` is false.

Parameters for individual nodes may be edited as well --- the most important being `serial_port` for both the range\_sensor\_driver and the `am_driver` nodes. 

### Run



### Start automower
- Turn on the robot with the switch below the handle.
- Open the hatch and input the pin: `1111`
    - If the automower has ben without power it will ask for a new pin, enter `1111`. I will also want to set up of time and date. Just press ok untill the menu is reached. 
- The loop detection should turn off automaticly if `main_2020.launch` has been launched. If not:
    - Enter settings and hold `7` and `9` until a new options appears.
    - Press the wrench then goto special settings -> override loop detection.
    - Move back to the main menu
    - Press start and close the hatch


### Troubleshooting
Here are some common errors and how to fix them:

`FATAL_ERROR`
Shown by hrp_teleop node and indicates that the automower is very sad. Open up the hatch and read the error message to proceed. It usually just want someone to listen to its problems.

`STOPPED`
Shown by hrp_teleop node and indicates that the automower is not started. If you are sure that it has started correctly try to restart the teleop node instead.

`SERIAL PORT DOESN'T EXIST`
Shown by am_driver_safe_node started by main_*.launch and indicates that the jetson tx1 cannot find the automower. Make sure that the usb cable is plugged in (hehe) and that ACM is listed under devices.



### Run simulation


