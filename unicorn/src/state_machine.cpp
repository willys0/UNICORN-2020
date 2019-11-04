#include "unicorn/state_machine.h"

StateMachine::StateMachine() : move_base_clt_("move_base", true)
{
    bool run_global_loc, sim_time;
    std::string odom_topic;
    if (!n_.getParam("use_sim_time", sim_time))
    {
        sim_time = false;
    }
    ROS_INFO("[UNICORN State Machine] sim_time: %i", sim_time);

    if (!n_.getParam("max_angular_vel", MAX_ANGULAR_VEL))
    {
        MAX_ANGULAR_VEL = 0.5;
    }
    ROS_INFO("[UNICORN State Machine] MAX_ANGULAR_VEL: %f", MAX_ANGULAR_VEL);
    if (!n_.getParam("max_linear_vel", MAX_LINEAR_VEL))
    {
        MAX_LINEAR_VEL = 0.3;
    }
    ROS_INFO("[UNICORN State Machine] MAX_LINEAR_VEL: %f", MAX_LINEAR_VEL);
    if (!n_.getParam("odometry_topic", odom_topic))
    {
        odom_topic = "odom";
    }
    if (!n_.getParam("frame_id", frame_id_))
    {
        frame_id_ = "base_link";
    }
    ROS_INFO("[unicorn_statemachine] Robot frame_id: %s", frame_id_.c_str());
    ROS_INFO("[unicorn_statemachine] Listening to %s", odom_topic.c_str());
    state_pub_ = n_.advertise<std_msgs::Int32>("/TX2_unicorn_state", 0);
    move_base_cancel_pub_ = n_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 0);
    amcl_global_clt_ = n_.serviceClient<std_srvs::Empty>("/global_localization");
    cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("/unicorn/cmd_vel", 0);
    odom_sub_ = n_.subscribe(odom_topic.c_str(), 0, &UnicornState::odomCallback, this);
    acc_cmd_srv_ = n_.advertiseService("cmd_charlie", &UnicornState::accGoalServer, this);
}

~StateMachine::StateMachine() {}

int StateMachine::start() {}

void StateMachine::cmdCallback(const std_msgs::String &msg) {}

cmd_struct StateMachine::parseCmdMsg(std::string cmd_msg) 
{
    json cmd_object = msg_json;
    cmd_struct new_cmd{
        cmd_object["state"].get<int>(),
        cmd_object["param1"].get<float>(),
        cmd_object["param2"].get<float>(),
        cmd_object["param3"].get<float>()};
    return new_cmd;
}

void updateAndPublishState(int new_state)
{
    state_msg_.data = new_state;
    unicorn_state_pub_.publish(state_msg_);
    return;
}
