#include <unicorn_state_machine/undock_state.h>

#include <unicorn_state_machine/idle_state.h>

UndockState::UndockState(int lift_cmd, ros::NodeHandle node) :
    State::State(node)
{
    lift_cmd_ = lift_cmd;
}

State* UndockState::run() {
    ROS_INFO("[UndockState] undock: %d", lift_cmd_);

    // TODO: Call the docking controller to initiate an undock

    // TODO: If failed to pickup or dropoff a bin, go to docking state
    // return new DockingState(lift_cmd_, nh_);

    return new IdleState(nh_);
}
