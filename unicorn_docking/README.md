# unicorn_docking

Package containing the implementation of the UNICORN docking controller. The controller will attempt to approach and align to an AprilTag mounted to a wall. At long distances camera images will be used to approximate distance and angle to the wall based on detected AprilTags, but as the robot closes in towards the wall LiDAR measurements will be used for more accurate approximations.

The node uses PID controllers from the `control_toolbox` package to control the velocity of the robot.

Most configurations of the controller are done using `dynamic_reconfigure`, however some configurations are done statically with parameters.

## Action API

*Action subscribed topics*

`~/dock_action/goal` - Sets a docking goal. A desired offset is provided as argument (NOT USED), as well as a boolean signifying if a undock should be carried out (if false, the controller will dock towards a bin station).

`~/dock_action/cancel` - Cancels the currently running docking action.

*Action published topics*

`~/dock_action/feedback` - The current errors of the docking action while running.

`~/dock_action/result` - The final errors of the docking action when finished, as well as a boolean whether the action succeeded or not.

## Subscribed topics

`/tf` - Used for getting transformations between detected apriltags, and frames of the robot.

`/frontLidar/scan` - Laser scan from the front LiDAR to be used to prevent collisions with obstacles when undocking.

`/rearLidar/scan` - Laser scan from the rear LiDAR for close-range distance and angle measurements, as well as for collision prevention.

`/realsense/color/image_raw` - The image from the realsense camera used to detect tags, which will be re-published on another topic when a docking action is active. This is used as a hack to stop the AprilTag node from performing any computations when not docking.

`/tag_detections` - AprilTag detections for publishing RViz markers. Only subscribed to when the `debug` parameter is true.

## Published topics

`/cmd_vel` - Output velocity when docking/undocking.

`~/errors` - The current errors of a docking action.

`/realsense/color/dock_image` - The re-published image from the realsense camera when docking. This is used as a hack to stop the AprilTag node from performing any computations when not docking.

## Parameters

`~/base_link_frame` - The tf frame which will be used to compare distance and rotation to, typically the center of the robot's wheel base.

`~/debug` - Whether to activate debug mode, where RViz markers will be published for detected AprilTags.

`~/front_lidar_angle` - The angle that objects are detected within by the front lidar 

`~/included_lidar_measures` - A list of LiDAR measurement indices that will be used distance and angle calculations.

`~/lidar_frame` - The tf frame of the rear LiDAR

`~/max_retries` - Number of retries (dock/undock cycles) before failing.

`~/max_time_since_lidar_scan` - The max number of seconds it can be since last laser scan

`~/pid/x` - Forward (x) PID settings to initialize the `control_toolbox` PID controller with.

`~/pid/th` - Rotational (th) PID settings to initialize the `control_toolbox` PID controller with.

`~/rear_lidar_angle` - The angle that objects are detected within by the rear LiDAR.

`~/retry_error_times` - Number of cycles to try to put robot within error margins before retrying.

`~/use_lidar` - Whether to include readings from the rear LiDAR when docking
