#include <unicorn_state_machine/dock_state.h>

#include <unicorn_state_machine/idle_state.h>
#include <unicorn_state_machine/lift_state.h>
#include <unicorn_state_machine/undock_state.h>

#include <actionlib/client/simple_action_client.h>

#include <unicorn_docking/DockAction.h>

typedef actionlib::SimpleActionClient<unicorn_docking::DockAction> DockActionClient;

DockState::DockState(int lift_cmd, ros::NodeHandle node) :
    State::State(node)
{
    lift_cmd_ = lift_cmd;
}

State* DockState::run() {
    ROS_INFO("[DockState] lift: %d", lift_cmd_);

    ROS_INFO("[DockState] Waiting for dock action server...");
    DockActionClient client("/dock", true);
    while(ros::ok() && !client.waitForServer(ros::Duration(0.5))) {
        RETURN_ON_ERROR();
    }

    // TODO: Try to find a tag, if fail go to idle state (?)
    // return new IdleState(nh_);

    // Call the docking controller to initiate a dock
    unicorn_docking::DockGoal goal;
    client.sendGoal(goal);
    while(ros::ok() && !client.waitForResult(ros::Duration(0.5))) {
        RETURN_ON_ERROR();
    }

    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        // Go to lift state if docked
        return new LiftState(lift_cmd_, nh_);
    }
    else {
        return new UndockState(lift_cmd_, nh_);
    }

    // TODO: Handle errors
    return new IdleState(nh_);

}
