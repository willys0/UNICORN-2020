# Unicorn

Core package for the UNICORN robot.

## Launching the UNICORN packages
The `main_2020.launch` file will run all nodes intended for the TX2 target, to launch the simulator use the `simulator_2020.launch` file instead.

By default SLAM is enabled. 
To use a pre-defined map set the argument `do_slam:=false`, and specify the path to the map with `map_file:=$(find <package>)/path/to/map.yaml`.

An rviz config with relevant displays is located in `config/default.rviz`. The `rviz.launch` file will start rviz and use this config. 
