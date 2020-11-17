#include <unicorn_state_machine/lift_state.h>

#include <unicorn_state_machine/undock_state.h>

#include <actionlib/client/simple_action_client.h>
#include <unicorn_roborio_bridge/RunLiftAction.h>

typedef actionlib::SimpleActionClient<unicorn_roborio_bridge::RunLiftAction> LiftActionClient;

LiftState::LiftState(int lift_cmd, ros::NodeHandle node) :
    State::State(node)
{
    lift_cmd_ = lift_cmd;
}

State* LiftState::run() {
    ROS_INFO("[LiftState] lift: %d", lift_cmd_);

    ROS_INFO("[LiftState] Waiting for lift action server...");
    LiftActionClient client("/lift/lift_action", true);
    client.waitForServer();

    unicorn_roborio_bridge::RunLiftGoal goal;

    if(lift_cmd_ == 1)
        goal.direction = goal.DIRECTION_PICKUP;
    else if(lift_cmd_ == 2)
        goal.direction = goal.DIRECTION_DROPOFF;
    else {
        // Should not even be able to get here
        return new UndockState(0, nh_);
    }

    // Call the lift with the correct lift_cmd
    client.sendGoal(goal);
    client.waitForResult();

    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        return new UndockState(0, nh_);
    }
    else if(client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
        // TODO: Handle preemptions
        return new UndockState(lift_cmd_, nh_);
    }
    else {
        // TODO: Handle unsuccessful pickups or drop-offs as announced by the RIO.
        return new UndockState(lift_cmd_, nh_);
    }

}
