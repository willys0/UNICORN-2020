# UNICORN LIDAR TRACKING PACKAGE

    This package is a prototyp that performs tracking with 2D Lidar data

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

    Parameters which can be adjusted with dynamic reconfig:

### Clustering
    lambda: 0.05 - Calibration value for clusters used in adaptive break-point detection, larger value will give finer clusters while smaller value allows for larger margins.
    max_dist_laser: 5.0 - Greatest distance allowed when detecting objects, this should be to a too high value.

### Static object filter -  removes objectles according to a predefined map
    static_filter: true - Enables/Disables static object filter.
    Static_map_removal_tolerance: 4 - Tolerance of filter in cells on occupancy grid.
    static_remove_ratio: 0.5 - Percentage of cluster that should be mark as occupied for the cluster to be removed.
### Shape Extraction
    polygon_tolerance: (defualt 1.02) - Tolerance in percent for shape extraction, lower tolerance will give finer shapes.
    polygon_side_min_points_required: (defualt 4) - Points required to contineu shape extraction.

### Association

    similarty_side_length_weight: (defualt 1.0) - Weight when comparing side lenght.
    similarty_side_angle_weigh: (defualt 1.0) - Weight for comparing angles between sides.
    similarty_side_amount_weight: (defualt 1.0) - Weight for comparing amount of sides.
    similarty_track_xposition_weight: (defualt 5.0) - Weight for comparing prediction x position to measured x position.
    similarty_track_yposition_weight: (defualt 5.0) - Weight for comparing prediction y position to measured y position.
    similarty_previous_position_weight: (defualt 2.0) - Weight for comparing previous position with current position.
    max_similarty_deviation: (defualt 2.0) - Maximum allowed value for deviation based on similarity index.
    min_size_cluster: (defualt 4) - Minimum points in cluster to count it in the association step.

### Tracker

    min_twist_detection: (defualt 0.3) - The minimum extimated veloctity that is needed the velocity to be shown in the visualiszation.
    TRACKER_LIFE: (defualt 50) - Frames that a tracker will live before removed if no matching object is found.
    CONFIRMED_TRACK: (defualt 20) - Frames that a object should be tracked before that tracker is confirmed.

### Frames
    mapframeid: (defualt map)
    odomframeid: (defualt odom_chassis)
    base_laser_frame: (defualt base_laser)
    base_frame: (defualt base_link)