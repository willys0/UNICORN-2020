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

    struct Goal goal = goals_[0];
    goals_.erase(goals_.begin());

    return new NavigatingState(goal.pose, goal.lift_cmd, nh_);
}
