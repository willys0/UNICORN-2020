#include <unicorn_state_machine/error_state.h>

#include <unicorn_state_machine/idle_state.h>

#include <actionlib_msgs/GoalID.h>

ErrorState::ErrorState() {

}

ErrorState::ErrorState(const std::string& error_msg, ros::NodeHandle node) :
    error_msg_(error_msg),
    State::State(node)
    {}

State* ErrorState::run() {

    ros::Publisher movebase_cancel_pub = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    ros::Publisher lift_cancel_pub = nh_.advertise<actionlib_msgs::GoalID>("/lift/lift_action/cancel", 1);

    actionlib_msgs::GoalID cancel_msg;

    do {
        ROS_INFO("[ErrorState] %s", error_msg_.c_str());

        movebase_cancel_pub.publish(cancel_msg);
        lift_cancel_pub.publish(cancel_msg);

        ros::Duration(0.5).sleep();
    } while(ros::ok() && error_);

    error_ = false;
    return new IdleState(nh_);

}
