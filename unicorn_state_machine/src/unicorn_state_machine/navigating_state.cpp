#include <unicorn_state_machine/navigating_state.h>

#include <unicorn_state_machine/idle_state.h>
#include <unicorn_state_machine/dock_state.h>
#include <unicorn_state_machine/error_state.h>

#include <geometry_msgs/PoseStamped.h>

NavigatingState::NavigatingState(const geometry_msgs::Pose& pose, int lift_cmd, ros::NodeHandle node) :
    State::State(node),
    movebase_client_("/move_base", true)
{
    goal_ = pose;
    lift_cmd_ = lift_cmd;
}

State* NavigatingState::run() {
    ROS_INFO("[NavigatingState] goal: %.2f, %.2f, lift: %d", goal_.position.x, goal_.position.y, lift_cmd_);

    geometry_msgs::PoseStamped ps;
    ps.pose = goal_;
    ps.header.frame_id = "map";
    ps.header.stamp = ros::Time::now();

    move_base_msgs::MoveBaseGoal mb_goal;
    mb_goal.target_pose = ps;

    ROS_INFO("[NavigatingState] Waiting for move_base server...");
    movebase_client_.waitForServer();

    ROS_INFO("[NavigatingState] Sending goal to move_base");
    movebase_client_.sendGoal(mb_goal);
    ROS_INFO("[NavigatingState] Waiting for move_base result");
    movebase_client_.waitForResult();

    ROS_INFO("[NavigatingState] Finished navigation");
    if(lift_cmd_ == 0) {
        return new IdleState(nh_);
    }

    if (movebase_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        if(lift_cmd_ != 0) {
            return new DockState(lift_cmd_, nh_);
        }
    }
    else {
        // We failed to navigate to the position for some reason.

        //  TODO: Handle move error
        return new ErrorState("Failed to navigate to goal position", nh_);
    }

    // TODO: What happens if we navigate to a pickup goal when we have failed to place
    // a bin back (i.e. the robot is currently holding a bin)?

    return new IdleState(nh_);
}
