# unicorn_intention

Publishes desired state of the LED strip and intention angle based on the current state of the robot and current `cmd_vel` respectively.

## Nodes

`unicorn_intention_node` - Main node

## Subscribed topics

`/unicorn/state` - Current state of the robot

`/cmd_vel` - Current velocity of the robot

## Published topics

`intention_angle` - The intended angle of movement in degrees.

`led_state` - Desired state of the LEDs, defined as (0) Idle, (1) Navigation, (2) Lifting, (3) Undocking, (4) Docking, (5) Error, (6) Emergency stop
