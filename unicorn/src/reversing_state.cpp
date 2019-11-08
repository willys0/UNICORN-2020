#include "unicorn/reversing_state.h"

REVERSINGState::REVERSINGState(ros::NodeHandle node) : move_base_clt_("move_base", false)
{
    cmd_vel_pub_ = node.advertise<geometry_msgs::Twist>("/unicorn/cmd_vel", 0);
    move_base_cancel_pub_ = node.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 0);
}

REVERSINGState::~REVERSINGState()
{
    
}


int REVERSINGState::sendGoal(Goal new_goal)
{    
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
    while (!move_base_clt_.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("[Unicorn State Machine] Waiting for the move_base action server to come up");
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

void REVERSINGState::cancelGoal()
{
    actionlib_msgs::GoalID cancel_all;
    move_base_cancel_pub_.publish(cancel_all);
    ROS_INFO("[Unicorn State Machine] Canceling move_base goal");
}

Command REVERSINGState::run()
{
    man_cmd_vel_.angular.z = 0;
    man_cmd_vel_.linear.x = -0.1;
    at_desired_distance_ = false;
    Command new_cmd;
    new_cmd.state = state_enum::IDLE;
    ROS_INFO("[Unicorn State Machine] Reversing towards to refuse bin...");
    ros::Rate rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        ROS_INFO("[Unicorn State Machine] Current velocity: %f", current_vel_);
        ROS_INFO("[Unicorn State Machine] Current yaw: %f", current_yaw_);
        if(command.state != -1)
        {
            cancelGoal();
            ROS_INFO("[Unicorn State Machine] New command was issued, halting navigation.");
            return command;
        }
        if(at_desired_distance_)
        {
            man_cmd_vel_.linear.x = 0.0;
            cancelGoal();
            cmd_vel_pub_.publish(man_cmd_vel_);
            ROS_INFO("[Unicorn State Machine] Robot reached desired distance exiting with new state set to LIFT");
            new_cmd.state = state_enum::LIFT;
            return new_cmd;
        }
        cmd_vel_pub_.publish(man_cmd_vel_);
        rate.sleep();
    }
}

void REVERSINGState::rearLidarCallback(const std_msgs::Float32 &msg)
{
    if(msg.data <= desired_distance_)
        at_desired_distance_ = true;

}

void REVERSINGState::odomCallback(const nav_msgs::Odometry &msg)
{
	tf::Pose pose;
	tf::poseMsgToTF(msg.pose.pose, pose);
	current_yaw_ = tf::getYaw(pose.getRotation());
	current_vel_ = msg.twist.twist.linear.x;
}
