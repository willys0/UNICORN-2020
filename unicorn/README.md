# Unicorn

Core package for the UNICORN robot, providing launch files to run the nodes of the robot's sub-systems.

**NOTE:** The launch files in this package does some major remapping of the topics provided and used by the UNICORN sub-packages, and if you are new to ROS this can be a little confusing. As such keep in mind that the topics may have different names when launching these files than those that are written in the sub-packages respective source (or README) files.

## Launching the UNICORN packages
The `main_2020.launch` file will run all nodes intended for the TX2 target, to launch the simulator use the `simulator_2020.launch` file instead.

By default SLAM is enabled. 
To use a pre-defined map set the argument `do_slam:=false`, and specify the path to the map with `map_file:=$(find <package>)/path/to/map.yaml`.

An rviz config with relevant displays is located in `config/default.rviz`. The `rviz.launch` file will start rviz and use this config. 
