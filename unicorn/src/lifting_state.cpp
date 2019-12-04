#include "unicorn/lifting_state.h"

LIFTState::LIFTState(ros::NodeHandle node) : move_base_clt_("move_base", false)
{
    lift_init_pub_ = node.advertise<std_msgs::Bool>("TX2_liftSubscriber_state", 1);
    lift_complete_sub_ = node.subscribe("RIO_publisher_masterMessage", 0, &LIFTState::liftCallback, this);
    cmd_vel_pub_ = node.advertise<geometry_msgs::Twist>("/unicorn/cmd_vel", 0);
    move_base_cancel_pub_ = node.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 0);
    state_identifier_ = STATE_LIFT;
}

LIFTState::~LIFTState()
{

}

Command LIFTState::run()
{
    std_msgs::Bool msg;
    msg.data = true;
    Command new_cmd;
    new_cmd.state = STATE_IDLE;
    ROS_INFO("[UNICORN State Machine] Lifting refuse bin, awaiting completion signal...");
    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        if (command.state != -1)
        {
            cancelGoal();
            ROS_INFO("[UNICORN State Machine] New command was issued, halting navigation.");
            return command;
        }
        if (lift_complete_)
        {
            break;
        }
       rate.sleep();
    }
    reverseFromBin();
    ROS_INFO("[UNICORN State Machine] Exiting to IDLE state.");
    return new_cmd;
}

void LIFTState::liftCallback(const unicorn::masterMessage &recieveMsg)
{ 
    if (recieveMsg.liftState == 5)
    {
	ROS_INFO("[UNICORN State Machine] Lift completion signal recieved.");
        lift_complete_ = true;
    }
}

void LIFTState::reverseFromBin()
{
    ROS_INFO("[UNICORN State Machine] Reversing from the refuse bin.");
    man_cmd_vel_.angular.z = 0;
    man_cmd_vel_.linear.x = 0.15;
    ros::Rate rate(10);
    int i = 0;
    while (i < 2)
    {
        cmd_vel_pub_.publish(man_cmd_vel_);
        i++;
        rate.sleep();
    }
    man_cmd_vel_.linear.x = 0.0;
    cancelGoal();
    cmd_vel_pub_.publish(man_cmd_vel_);
    return;
}

void LIFTState::cancelGoal()
{
    actionlib_msgs::GoalID cancel_all;
    move_base_cancel_pub_.publish(cancel_all);
    ROS_INFO("[UNICORN State Machine] Canceling move_base goal");
}
