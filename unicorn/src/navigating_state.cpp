#include "unicorn/navigating_state.h"

NAVIGATINGState::NAVIGATINGState(float x, float y, float yaw, ros::NodeHandle node) : move_base_clt_("move_base", false)
{
    current_goal_.x = x;
    current_goal_.y = y;
    current_goal_.yaw = yaw;
    ROS_INFO("[UNICORN State Machine] New goal set to x=%f, y=%f, yaw=%f", x, y, yaw);
    node_ = node;
    move_base_cancel_pub_ = node_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 0);
    state_identifier_ = STATE_NAVIGATING;
}

NAVIGATINGState::~NAVIGATINGState()
{

}

int NAVIGATINGState::sendGoal(Goal new_goal)
{    
    int attempts = 0;
    try
    {
        float check_input = boost::lexical_cast<float>(new_goal.x);
    }
    catch (boost::bad_lexical_cast &e)
    {
        ROS_INFO("%s", e.what());
        ROS_ERROR("[Unicorn State Machine] x is undefined");
        return -1;
    }
    try
    {
        float check_input = boost::lexical_cast<float>(new_goal.y);
    }
    catch (boost::bad_lexical_cast &)
    {
        ROS_ERROR("[Unicorn State Machine] y is undefined");
        return -1;
    }
    try
    {
        float check_input = boost::lexical_cast<float>(new_goal.yaw);
    }
    catch (boost::bad_lexical_cast &)
    {
        ROS_ERROR("[Unicorn State Machine] yaw is undefined");
        return -1;
    }
    while (!move_base_clt_.waitForServer(ros::Duration(5.0)) && (attempts < 4))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
        attempts++;
    }
    if(attempts == 3)
    {
        ROS_WARN("Failed to contact move base server after four attempts, goal not published!");
        return -1;
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

void NAVIGATINGState::cancelGoal()
{
    actionlib_msgs::GoalID cancel_all;
    move_base_cancel_pub_.publish(cancel_all);
    ROS_INFO("[Unicorn State Machine] Canceling move_base goal");
}

Command NAVIGATINGState::run()
{
    Command new_cmd;
    new_cmd.state = STATE_IDLE;
    int err = sendGoal(current_goal_);
    if(err == -1)
    {
        ROS_ERROR("[Unicorn State Machine] Could not send goal. Entering IDLE state.");
        return new_cmd;
    }
    ros::Rate rate(50);

    while (true)
    {
        ros::spinOnce();
        if(command.state != -1)
        {
            cancelGoal();
            ROS_INFO("[Unicorn State Machine] New command was issued, halting navigation.");
            return command;
        }
        if(move_base_clt_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("[Unicorn State Machine] Robot reached designated goal!");
            break;
        }
        rate.sleep();
    }
    return new_cmd;
}

