#include "unicorn/aligning_state.h"

ALIGNINGState::ALIGNINGState(ros::NodeHandle node, RefuseBin bin) : move_base_clt_("move_base", false)
{
    node_ = node;
    bin_location_ = bin;
    move_base_cancel_pub_ = node_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 0);
    state_identifier_ = STATE_ALIGNING;
}

ALIGNINGState::~ALIGNINGState()
{
    
}

void ALIGNINGState::cancelGoal()
{
    actionlib_msgs::GoalID cancel_all;
    move_base_cancel_pub_.publish(cancel_all);
    ROS_INFO("[UNICORN State Machine] Canceling move_base goal");
}

int ALIGNINGState::sendGoal(Goal new_goal)
{    
    try
    {
        float check_input = boost::lexical_cast<float>(new_goal.x);
    }
    catch (boost::bad_lexical_cast &e)
    {
        ROS_INFO("%s", e.what());
        ROS_ERROR("[UNICORN State Machine] x is undefined");
        return -1;
    }
    try
    {
        float check_input = boost::lexical_cast<float>(new_goal.y);
    }
    catch (boost::bad_lexical_cast &)
    {
        ROS_ERROR("[UNICORN State Machine] y is undefined");
        return -1;
    }
    try
    {
        float check_input = boost::lexical_cast<float>(new_goal.yaw);
    }
    catch (boost::bad_lexical_cast &)
    {
        ROS_ERROR("[UNICORN State Machine] yaw is undefined");
        return -1;
    }
    while (!move_base_clt_.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("[UNICORN State Machine] Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = new_goal.x;
    goal.target_pose.pose.position.y = new_goal.y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(new_goal.yaw);

    move_base_clt_.sendGoal(goal);

    return 1;
}

Command ALIGNINGState::run()
{
    ros::Rate rate(50);
    ROS_INFO("[UNICORN State Machine] Aligning with garbage disposal...");
    Command new_cmd;
    new_cmd.state = STATE_IDLE;
    Goal new_goal;
    new_goal.x = bin_location_.x + 1.5 * cos(bin_location_.yaw);
    new_goal.y = bin_location_.y + 1.5 * sin(bin_location_.yaw);
    new_goal.yaw = bin_location_.yaw;
    sendGoal(new_goal);
    while (true)
    {
        ros::spinOnce();
        if(command.state != -1)
        {
            cancelGoal();
            ROS_INFO("[UNICORN State Machine] New command was issued, halting alignment.");
            return command;
        }
        if(move_base_clt_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("[UNICORN State Machine] Robot has aligned with refuse bin, transmitting LIFT command.");
            new_cmd.state = STATE_LIFT;
            return new_cmd;
        }
        rate.sleep();
    }
}
