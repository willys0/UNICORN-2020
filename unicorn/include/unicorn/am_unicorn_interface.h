/** 
*	@file am_unicorn_interface.h
*	@Author Alexander Karlsson (akn13013@student.mdh.se)
*/
#ifndef AM_UNICORN_INTERFACE_H
#define AM_UNICORN_INTERFACE_H

/* ROS */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

/* C / C++ */
#include <iostream>
#include <termios.h>

/** @brief Main class for automower to unicorn interface node
*		
*	Flips the velocity command from topic "/unicorn/cmd_vel" and publishes on
*	"cmd_vel".
*/
class AmUnicornInterface
{
public:
	AmUnicornInterface();
	void cmdVelCallback(const geometry_msgs::Twist& cmd_vel);
	void publishCmd();
private:
	ros::NodeHandle n_;
	geometry_msgs::Twist unicorn_cmd_vel_;
	ros::Subscriber cmd_vel_sub_;
	ros::Publisher unicorn_cmd_vel_pub_;
};
#endif