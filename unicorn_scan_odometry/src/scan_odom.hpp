#ifndef SCAN_ODOM_H
#define SCAN_ODOM_H

/* ROS */
#include <ros/ros.h>
#include <tf/tf.h>

#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <unicorn_scan_odometry/scan_odomConfig.h>

/* C / C++ */
#include <iostream>
#include <termios.h>
#include <math.h>
#include <stdio.h>
#include <cstdint>
#include <new>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include "ICP/icpPointToPlane.h"


class scan_odom
{
public:
	scan_odom();
	void scanCallback(const sensor_msgs::LaserScan& scan);
	void esitmate_odometry();
	float outlier_ratio;

	geometry_msgs::TransformStamped Lidar2base;

	std::string odomframeid = "odom_chassis";
	std::string base_laser_frame = "base_laser";
	std::string base_frame = "chassis_link";
	
  	
private:

	ros::NodeHandle n_;

	nav_msgs::Odometry esitmated_odometry;

	sensor_msgs::LaserScan scan_data_;
	sensor_msgs::LaserScan scan_data_old;

	ros::Subscriber scan_sub_;
	ros::Publisher est_odom_pub_;

	tf2_ros::Buffer tf_buffer,tf_buffer2;
	int seq;
	bool scan_received = false;
	
};
#endif

