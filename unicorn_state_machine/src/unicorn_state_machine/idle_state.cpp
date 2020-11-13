#include <unicorn_state_machine/idle_state.h>

#include <unicorn_state_machine/navigating_state.h>

IdleState::IdleState() {
   
}

IdleState::IdleState(ros::NodeHandle node) :
    State::State(node)
    {}

State* IdleState::run() {

    ROS_INFO("[IdleState]");

    geometry_msgs::Pose goal;
    goal.orientation.w = 1;

    return new NavigatingState(goal, 1, nh_);
}
