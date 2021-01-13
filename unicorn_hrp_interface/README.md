# unicorn_hrp_interface

Main interface between the Husqvarna Automower research platform and the rest of the UNICORN nodes. Flips the signs of the x (forward) and z (angular) components of provided `/unicorn/cmd_vel` messages to drive the mower "backwards", and outputs a modified `/cmd_vel`.

Implements additional safety features such as slowing down and stopping movement if desired velocity is not provided at a certain frequency (to prevent cases where a controlling joystick loses connection etc.). Also disables any actions running on lift, docking controller, or in move_base when either the stop button on the mower or the emergency stop is pressed.

## Nodes
`unicorn_hrp_interface_node` - Main node

## Subscribed topics

`/unicorn/cmd_vel` - Desired velocity from UNICORN nodes

### hrp topics
`/sensor_status` - Listens to the internal state of the mower to stop other UNICORN actions in case stop button (and wireless emergency stop) is activated.


## Published topics

`/cmd_vel` - Final velocity sent to the automower

`/move_base/cancel` - Cancels navigation commands on stop button pressed

`/dock/cancel` - Cancels dock commands on stop button pressed

`/lift/lift_action/cancel` - Cancels lift commands on stop button pressed

### hrp topics
`/cmd_mode` - Sets mode of automower to disable loop signal if it is activated.

## Parameters
`max_velocity_delay` - Maximum delay between desired velocity messages before stopping the mower.
