#ifndef TRACKING_LIDAR_H
#define TRACKING_LIDAR_H

/* ROS */
#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Polygon.h>

#include <costmap_converter/ObstacleArrayMsg.h>
#include <costmap_converter/ObstacleMsg.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <unicorn_tracking/TrackingConfig.h>
#include <tf/transform_listener.h>


/* C / C++ */
#include <iostream>
#include <termios.h>
#include <math.h>
#include <stdio.h>
#include <cstdint>
#include <new>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

/* Kalman */
#include "kalman/kalman.hpp"
#include "Hungarian/Hungarian.h"
#include "shape_extraction.hpp"
#include "association.hpp"

#define PI 3.14159265
#define Lidar_Height 0.465
#define MAX_OBJECTS 100
#define SCAN_SIZE 800
#define MAXTRACKS MAX_OBJECTS

class tracking_lidar
{
public:
	tracking_lidar();
	void odomCallback(const nav_msgs::Odometry& odometry);
	void odomCallback2(const nav_msgs::Odometry& odometry);
	void mapCallback(const nav_msgs::OccupancyGrid& map);
	void scanCallback(const sensor_msgs::LaserScan& scan);
	void publishmsg();
	void object_publisher();


	association association_interface;
	/* Variables that can be dynamically changed */
	float lambda;
	float max_dist_laser;
	int static_remove_dist;
	float static_remove_ratio;
	float polygon_tolerance;
	int polygon_min_points, min_size_cluster;


	float min_twist_detection, max_similarty_deviation;
	float sim_adj_dist, sim_adj_angle, sim_adj_side, sim_adj_xpos, sim_adj_ypos,sim_adj_posdiff;


	bool static_filter;


	int TRACKER_LIFE; 
	int CONFIRMED_TRACK;


	std::string mapframeid = "map";
	std::string odomframeid = "odom_chassis";
	std::string base_laser_frame = "base_laser";
	std::string base_frame = "chassis_link";
	tf2_ros::Buffer tf_buffer,tf_buffer2,tf_buffer3;

	struct association_variables{
		//similarity weights
		float *sim_adj_dist;
		float *sim_adj_angle;
		float *sim_adj_side;
		float *sim_adj_xpos;
		float *sim_adj_ypos;
		float *sim_adj_posdiff;
		float *max_similarty_deviation;

	}typedef association_variables;


	shape_extraction shape_interface;
  	
private:
	geometry_msgs::Point transform_point_odometry(geometry_msgs::Point position, const nav_msgs::Odometry& odometryData_old,const nav_msgs::Odometry& odometryData_new);
	geometry_msgs::Point transform_vel_odometry(geometry_msgs::Point position, const nav_msgs::Odometry& odometryData_old,const nav_msgs::Odometry& odometryData_new);


	ros::NodeHandle n_;
	nav_msgs::Odometry odometry_data_; 
	nav_msgs::Odometry odometry_data_2; 
	nav_msgs::Odometry stable_odom_; 
	nav_msgs::OccupancyGrid map_data_;
	sensor_msgs::LaserScan scan_data_;
	geometry_msgs::TransformStamped Lidar2base;
	geometry_msgs::TransformStamped odom2map;
	geometry_msgs::TransformStamped base2odom;
	ros::Subscriber odometry_sub_;
	ros::Subscriber odometry_sub_2;
	ros::Subscriber map_sub_;
	ros::Subscriber scan_sub_;
	ros::Publisher object_pub_;
	ros::Publisher marker_pub_;
	ros::Publisher marker_Arrow_pub_;
	ros::Publisher est_odom_pub_;
	int seq;

	bool map_received = false;
	bool odom_received = false;
	bool scan_received = false;
	bool stable_odom_received = false;
	
};
#endif

