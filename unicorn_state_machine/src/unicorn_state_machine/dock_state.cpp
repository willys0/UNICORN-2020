#include <unicorn_state_machine/dock_state.h>

#include <unicorn_state_machine/idle_state.h>
#include <unicorn_state_machine/lift_state.h>

DockState::DockState(int lift_cmd, ros::NodeHandle node) :
    State::State(node)
{
    lift_cmd_ = lift_cmd;
}

State* DockState::run() {
    ROS_INFO("[DockState] lift: %d", lift_cmd_);

    // TODO: Try to find a tag, if fail go to idle state (?)
    // return new IdleState(nh_);

    // TODO: Call the docking controller to initiate a dock

    // TODO: If failed to dock (define failed), do an undock
    // return new UndockState(lift_cmd_, nh_);

    // TODO: Go to lift state if docked
    return new LiftState(lift_cmd_, nh_);

}
