#include <unicorn_state_machine/undock_state.h>

#include <unicorn_state_machine/idle_state.h>
#include <unicorn_state_machine/dock_state.h>

#include <actionlib/client/simple_action_client.h>

#include <unicorn_docking/DockAction.h>

typedef actionlib::SimpleActionClient<unicorn_docking::DockAction> DockActionClient;

UndockState::UndockState(int lift_cmd, ros::NodeHandle node) :
    State::State(node)
{
    lift_cmd_ = lift_cmd;
}

State* UndockState::run() {
    ROS_INFO("[UndockState] undock: %d", lift_cmd_);

    ROS_INFO("[UndockState] Waiting for dock action server...");
    DockActionClient client("/dock", true);
    while(ros::ok() && !client.waitForServer(ros::Duration(0.5))) {
        RETURN_ON_ERROR();
    }

    // TODO: Call the docking controller to initiate an undock
    unicorn_docking::DockGoal goal;
    goal.undock = true;
    client.sendGoal(goal);
    while(ros::ok() && !client.waitForResult(ros::Duration(0.5))) {
        RETURN_ON_ERROR();
    }

    // TODO: If failed to pickup or dropoff a bin, go to docking state
    // return new DockingState(lift_cmd_, nh_);

    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        if(lift_cmd_ != 0) {
            // Go to lift state if failed to pick up bin
            return new DockState(lift_cmd_, nh_);
        }
        else {
            return new IdleState(nh_);
        }
    }

    return new IdleState(nh_);
}
