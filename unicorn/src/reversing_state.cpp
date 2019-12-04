#include "unicorn/reversing_state.h"

REVERSINGState::REVERSINGState(ros::NodeHandle node) : move_base_clt_("move_base", false)
{
    cmd_vel_pub_ = node.advertise<geometry_msgs::Twist>("/unicorn/cmd_vel", 0);
    move_base_cancel_pub_ = node.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 0);
    rear_lidar_sub_ = node.subscribe("RIO_publisher_masterMessage", 10, &REVERSINGState::rearLidarCallback, this);
    state_identifier_ = STATE_REVERSING;
    at_desired_distance_ = false;
    man_cmd_vel_.angular.z = 0;
    man_cmd_vel_.linear.x = -0.1;
}

REVERSINGState::~REVERSINGState()
{
    
}

void REVERSINGState::cancelGoal()
{
    actionlib_msgs::GoalID cancel_all;
    move_base_cancel_pub_.publish(cancel_all);
    ROS_INFO("[UNICORN State Machine] Canceling move_base goal");
}

Command REVERSINGState::run()
{

    Command new_cmd;
    new_cmd.state = STATE_IDLE;
    ROS_INFO("[UNICORN State Machine] Reversing towards to refuse bin...");
    ros::Rate rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
//        ROS_INFO("[UNICORN State Machine] Current velocity: %f", current_vel_);
//        ROS_INFO("[UNICORN State Machine] Current yaw: %f", current_yaw_);
        if(command.state != -1)
        {
            man_cmd_vel_.linear.x = 0.0;
            cmd_vel_pub_.publish(man_cmd_vel_);
            cancelGoal();
            ROS_INFO("[UNICORN State Machine] New command was issued, halting navigation.");
            new_cmd.state = command.state;
            new_cmd.param1 = command.param1;
            new_cmd.param2 = command.param2;
            new_cmd.param3 = command.param3;
            break;
        }
        if(at_desired_distance_)
        {
            man_cmd_vel_.linear.x = 0.0;
            cancelGoal();
            cmd_vel_pub_.publish(man_cmd_vel_);
            ROS_INFO("[UNICORN State Machine] Robot reached desired distance exiting with new state set to LIFT");
            new_cmd.state = STATE_LIFT;
            break;
        }
        cmd_vel_pub_.publish(man_cmd_vel_);
        rate.sleep();
    }
    return new_cmd;
}

void REVERSINGState::rearLidarCallback(const unicorn::masterMessage &msg)
{
    if(msg.avgDist <= desired_distance_ && msg.avgDist != 0.0)
    {
        at_desired_distance_ = true;
    }
    
    else
    {
	    ROS_INFO("Current Distance: %f", msg.avgDist);
    }
    
}

void REVERSINGState::odomCallback(const nav_msgs::Odometry &msg)
{
	tf::Pose pose;
	tf::poseMsgToTF(msg.pose.pose, pose);
	current_yaw_ = tf::getYaw(pose.getRotation());
	current_vel_ = msg.twist.twist.linear.x;
}
