#include <unicorn_state_machine/lift_state.h>

#include <unicorn_state_machine/undock_state.h>

LiftState::LiftState(int lift_cmd, ros::NodeHandle node) :
    State::State(node)
{
    lift_cmd_ = lift_cmd;
}

State* LiftState::run() {
    ROS_INFO("[LiftState] lift: %d", lift_cmd_);

    // TODO: Call the lift with the correct lift_cmd

    // TODO: Handle unsuccessful pickups or drop-offs as announced by
    // the RIO.

    return new UndockState(0, nh_);
}
