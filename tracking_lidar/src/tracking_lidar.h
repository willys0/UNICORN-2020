#ifndef TRACKING_LIDAR_H
#define TRACKING_LIDAR_H

/* ROS */
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>



#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* C / C++ */
#include <iostream>
#include <termios.h>

class tracting_lidar
{
public:
	tracting_lidar();
	void odomCallback(const nav_msgs::Odometry& odometry);
	void mapCallback(const nav_msgs::OccupancyGrid& map);
	void scanCallback(const sensor_msgs::LaserScan& scan);
	void publishmsg();
private:
	ros::NodeHandle n_;
	nav_msgs::Odometry odometry_data_;
	nav_msgs::OccupancyGrid map_data_;
	sensor_msgs::LaserScan scan_data_;
	ros::Subscriber odometry_sub_;
	ros::Subscriber map_sub_;
	ros::Subscriber scan_sub_;

};
#endif


/*
#ifndef ODOMETRY_TRANSFORM_H
#define ODOMETRY_TRANSFORM_H


#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer_interface.h>


#include <iostream>
#include <termios.h>

class odom_transform
{
public:
	odom_transform();
	void msgCallback(const nav_msgs::Odometry& odometry);
	void publishmsg();
private:
	ros::NodeHandle n_;
	nav_msgs::Odometry odometry_data_;
	ros::Subscriber odometry_sub_;
	ros::Publisher odometry_transform_pub_;
};
#endif
*/