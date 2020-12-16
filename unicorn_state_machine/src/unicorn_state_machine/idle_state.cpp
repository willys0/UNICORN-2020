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

    struct Goal goal;

    if(!getNextGoal(goal)) {
        if(repeating_)
            goal_it_.reset();

        while(!getNextGoal(goal)) {
            if(!ros::ok())
                return nullptr;

            RETURN_ON_ERROR();
            ROS_INFO("[IdleState] No goals, sleeping for a while...");
            ros::Duration(2.0).sleep();
        }
    }

    return new NavigatingState(goal.pose, goal.lift_cmd, nh_);
}
