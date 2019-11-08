#include "unicorn/lifting_state.h"

LIFTState::LIFTState(ros::NodeHandle node) : move_base_clt_("move_base", false)
{
    lift_init_pub_ = node.advertise<std_msgs::Bool>("/TX2_liftPublisher_state", 0);
    lift_complete_sub_ = node.subscribe("/RIO_liftSubcriber_state", 0, &LIFTState::liftCallback, this);
    cmd_vel_pub_ = node.advertise<geometry_msgs::Twist>("/unicorn/cmd_vel", 0);
    move_base_cancel_pub_ = node.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 0);
}

LIFTState::~LIFTState()
{

}

Command LIFTState::run()
{
    std_msgs::Bool msg;
    msg.data = true;
    Command new_cmd;
    new_cmd.state = state_enum::IDLE;
    ROS_INFO("[Unicorn State Machine] Lifting refuse bin, awaiting completion signal...");
    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        if(command.state != -1)
        {
            cancelGoal();
            ROS_INFO("[Unicorn State Machine] New command was issued, halting navigation.");
            return command;
        }
        if(lift_complete_)
        {
            break;
        }
        lift_init_pub_.publish(msg);
        rate.sleep();
    }

    reverseFromBin();
    return new_cmd;
}

void LIFTState::liftCallback(const std_msgs::Bool &recieveMsg)
{
    if(recieveMsg.data)
    {
        lift_complete_ = true;
    }
}

void LIFTState::reverseFromBin()
{
    ros::Rate rate(1);
    ROS_INFO("[Unicorn State Machine] Reversing from the refuse bin.");
    man_cmd_vel_.angular.z = 0;
    man_cmd_vel_.linear.x = 0.15;
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
    ROS_INFO("[Unicorn State Machine] Canceling move_base goal");
}