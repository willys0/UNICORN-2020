# Unicorn Quick Start Guide

### Materials

- Jetson TX1
- LMS111
- Zed Camera || Orbbec
- Usb 3.0 hub
- Husqvarna Automower

### Connection

Make sure that you are able to create an ssh tunnel between your host computer and the target jetsontx1.

> ssh nvidia@10.42.0.1

If not, check if you are connected to unicorn-hotspot and try again.

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

(*) end with "zed" or "orbbec" depending on which camera is in use.

### Run

Make two terminals and ssh from both to the jetson tx1 (you may run the teleop from host as well with ROS_IP and ROS_MASTER_URI configured).
 
In the first one run:

> cd

> ./lidar_connect.sh

> roslaunch unicorn main_*.launch

- In the second:

> rosrun unicorn teleop.launch

- Make another terminal and configure `ROS_IP` and `ROS_MASTER_URI` to enable reading topics from the jetson tx1.

> export ROS_IP=$YOUR_IP

> export ROS_MASTER_URI=`http://10.42.0.1:11311`

Then start the interface to the unicorn state machine and (optionally) rviz:

> rosrun unicorn unicorn_statemachine

> rosrun rviz rviz

### Start automower

- Begin by opening the hatch and input the pin: `1111`
- Enter settings and hold `7` and `9` until a new options appears.
    - Press the wrench then goto special settings -> line something? and press the down arrow to disable.
- Start by moving back to the main menu
    + Press start and close the hatch


### Troubleshooting
Here are some common errors and how to fix them:

`FATAL_ERROR`
Shown by hrp_teleop node and indicates that the automower is very sad. Open up the hatch and read the error message to proceed. It usually just want someone to listen to its problems.

`STOPPED`
Shown by hrp_teleop node and indicates that the automower is not started. If you are sure that it has started correctly try to restart the teleop node instead.

`SERIAL PORT DOESN'T EXIST`
Shown by am_driver_safe_node started by main_*.launch and indicates that the jetson tx1 cannot find the automower. Make sure that the usb cable is plugged in (hehe) and that ACM is listed under devices.

> ls /dev/ttyACM*

Compare the number behind ACM with the number in `main_*.launch` and edit if needed.

`LIDAR NOT PUBLISHING /SCAN`
Make sure that the variables `ROS_IP` and `ROS_MASTER_URI` are set correctly on your host computer.

> echo $ROS_IP && echo $ROS_MASTER_URI

Otherwise make sure that the ip address of the LMS111 is set correctly.

> nmap -sP 192.168.0.2/24

May take a few seconds. If it doesn't output anything try to investigate if our link to eth0 is up using:

> ifconfig

And if it is down set it up using:

> sudo ip link set eth0 up

Then repeat the nmap and if you find the ip address (typically 192.168.0.100) then compare it to the one set in `main_*.launch` under "Start the LMS111...".

If it's still not working it may be a hardware issue so go check if the eth cable led is blinking and if the lidar is not working. You can check this by looking at the colored lights at the front of the lidar where green indicates that it's ok, red the opposite, and yellow shows that you need to wipe the lidar. Checking documentation for this is recommended.

### Run simulation

> roslaunch unicorn unicorn_sim.launch

`Note:` Keep in mind that changing the model used may require you to comment out the line `<node name="am_unicorn_interface" pkg="unicorn" type="am_unicorn_interface" output="screen"/>` if the model is forward driven.

If you want to run the unicorn simulation with camera and lidar fusion costmaps use

> roslaunch unicorn_slam unicorn_sim_camera_mapping.launch 

or if you want to use the less computationally intense version use

> roslaunch unicorn_slam unicorn_sim_camera_mapping_filtered_cloud.launch

the filtered version uses a voxel_grid to filter the output pointcloud before using it for the costmap.
