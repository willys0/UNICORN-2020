#ifndef TRACKING_LIDAR_H
#define TRACKING_LIDAR_H

/* ROS */
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/* C / C++ */
#include <iostream>
#include <termios.h>
#include <math.h>
#include <stdio.h>
#include <cstdint>
#include <new>

#define PI 3.14159265
#define MAX_OBJECTS 100

class tracting_lidar
{
public:
	tracting_lidar();
	void odomCallback(const nav_msgs::Odometry& odometry);
	void mapCallback(const nav_msgs::OccupancyGrid& map);
	void scanCallback(const sensor_msgs::LaserScan& scan);
	void publishmsg();
	void adaptive_breaK_point();
	void static_map_filter();
	void polygon_extraction();
private:
	void extract_corners(int startpoint,int endpoint, int length,int shape_nr);
	void search_longest(int startpoint, int current_point,int end_point, int length, float distance_S, int itteration, int max_itteration, int *best_point, float *best_dist);

	ros::NodeHandle n_;
	nav_msgs::Odometry odometry_data_;
	nav_msgs::OccupancyGrid map_data_;
	sensor_msgs::LaserScan scan_data_;
	geometry_msgs::Polygon shapes[MAX_OBJECTS];
	ros::Subscriber odometry_sub_;
	ros::Subscriber map_sub_;
	ros::Subscriber scan_sub_;

	float lambda;
	int max_dist_laser;
	int static_remove_dist;
	float polygon_tolerance;
	int polygon_min_points;


	float xy_positions[800][2];
	float xy_map_positions[800][2];
  	int clusters[800];
	int polygon[800];
	int polygon_size[MAX_OBJECTS];
	  
	double roll, pitch, yaw;
	double x,y,z;
	uint32_t mapx,mapy;


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