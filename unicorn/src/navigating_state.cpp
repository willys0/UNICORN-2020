#include "unicorn/navigating_state.h"

NAVIGATINGState::NAVIGATINGState(float x, float y, float yaw, ros::NodeHandle node) : move_base_clt_("move_base", true)
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
    while (!move_base_clt_.waitForServer(ros::Duration(5.0)) && (attempts < 4))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
        attempts++;
    }
    if(attempts >= 3)
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
    ROS_INFO("[UNICORN State Machine] Canceling move_base goal");
}

Command NAVIGATINGState::run()
{
    Command new_cmd;
    new_cmd.state = STATE_IDLE;
    int ret = sendGoal(current_goal_);
    if(ret == -1)
    {
        ROS_ERROR("[UNICORN State Machine] Could not send goal. Entering IDLE state.");
        return new_cmd;
    }
    actionlib::SimpleClientGoalState goalState = move_base_clt_.getState();
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
        goalState = move_base_clt_.getState();
        if(goalState == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("[Unicorn State Machine] Robot reached designated goal!");
            break;
        }
        rate.sleep();
    }
    return new_cmd;
}

