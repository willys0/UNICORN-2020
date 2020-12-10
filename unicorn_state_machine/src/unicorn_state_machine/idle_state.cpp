#include <unicorn_state_machine/idle_state.h>

#include <unicorn_state_machine/navigating_state.h>

#include <unicorn_state_machine/goal.h>

IdleState::IdleState() {
   
}

IdleState::IdleState(ros::NodeHandle node) :
    State::State(node)
    {}

State* IdleState::run() {

    ROS_INFO("[IdleState]");

    while(ros::ok() && goals_.empty()) {
        RETURN_ON_ERROR();
        ROS_INFO("[IdleState] No goals, sleeping for a while...");
        ros::Duration(2.0).sleep();
    }

    struct Goal goal = *(goals_.end() - 1);
    goals_.pop_back();

    return new NavigatingState(goal.pose, goal.lift_cmd, nh_);
}
