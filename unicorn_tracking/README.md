# UNICORN LIDAR TRACKING PACKAGE

This package preforms tracking with Lidar data

Lidar data is clustered with Adaptive Break-Point Detection and tracked using kalman filters  


![Unicorn](docs/tracking.png)

## Subcribed Topics

    /odometry/filtered - Used to relate the robots position to the map

    /scan - 2D Lidar scan 

    /wheel_encoder/odom - Stable odometry source which is good to estimate movement, this odometry should not contain any sharp moment that odometry based on global coordinates as GPS can supply. Wheelencoders are a good source for this.

## Published Topics 

    obstacles - costmap_converter/ObstacleArrayMsg message with all detected dynamic obstacles.

    markerArray - visualization_msgs/MarkerArray message which visualizes the obstacles in rviz.

    markerArrowArray - visualization_msgs/MarkerArray message which visualizes the direct with the obstacle is moving in rviz.


## Changable Parameters 


### Clustering
lambda: 0.05
max_dist_laser: 5.0

### Static object filter
static_filter: true
Static_map_removal_tolerance: 4

### Shape Extraction
polygon_tolerance: 1.02 # 1.04
static_remove_ratio: 0.5
polygon_side_min_points_required: 4

### Association
similarty_side_length_weight (defualt 1.0)
similarty_side_angle_weigh: (defualt 1.0)
similarty_side_amount_weight: (defualt 1.0)
similarty_track_xposition_weight: (defualt 5.0)
similarty_track_yposition_weight: (defualt 5.0)
similarty_previous_position_weight: (defualt 2.0)
max_similarty_deviation: (defualt 2.0)
min_size_cluster: (defualt 4)

### Tracker

min_twist_detection: (defualt 0.3)
TRACKER_LIFE: (defualt 50)
CONFIRMED_TRACK: (defualt 20)

### Frames
mapframeid: (defualt map)
odomframeid: (defualt odom_chassis)
base_laser_frame: (defualt base_laser)
base_frame: (defualt base_link)