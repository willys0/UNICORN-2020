# unicorn_roborio_bridge

Main interface to the RoboRIO platform. Reads the data from the master message sent from the RIO (R2100 LiDAR readings, lift state, UWB pos) and publishes it to different topics to be used by UNICORN nodes.

Also provides an actionlib interface for interacting with the lift.

## Nodes

`unicorn_roborio_bridge_node` - Main node.

`unicorn_roborio_bridge_dummy_data_node` - Publishes messages with dummy data to `/RIO_publisher` to simulate the RoboRIO.

## Action API

`/lift/lift_action/goal` - A routine to be executed by the lift, either pickup or dropoff. 

`/lift/lift_action/cancel` - Request to cancel running lift action to stop the lift.

## Subscribed topics

`/RIO_publisher` - The topic to where the RIO publishes its master message.

## Published topics

`/uwb/pose` - Pose from UWB (only position for now, no orientation).

`/lift/state` - Current state of the lift.

`/lift/picking_routine` - Requested lift routine to be read by the RoboRIO.

`/rearLidar/scan` - Laser scan from the rear mounted R2100 LiDAR.
