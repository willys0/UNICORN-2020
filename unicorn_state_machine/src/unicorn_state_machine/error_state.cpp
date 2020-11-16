#include <unicorn_state_machine/error_state.h>

#include <unicorn_state_machine/idle_state.h>

ErrorState::ErrorState() {

}

ErrorState::ErrorState(const std::string& error_msg, ros::NodeHandle node) :
    error_msg_(error_msg),
    State::State(node)
    {}

State* ErrorState::run() {

    while(ros::ok()) {
        ROS_INFO("[ErrorState] %s", error_msg_.c_str());

        ros::Duration(0.5).sleep();

    }

    return new IdleState(nh_);

}
