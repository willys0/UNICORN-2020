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
#define MAX_OBJECTS 100
#define SCAN_SIZE 800
#define MAXTRACKS MAX_OBJECTS

class tracking_lidar
{
public:
	tracking_lidar();
	void odomCallback(const nav_msgs::Odometry& odometry);
	void wheelodomCallback(const nav_msgs::Odometry& odometry); 
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
	tf2_ros::Buffer tf_buffer;

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
	geometry_msgs::Point32 transform_point(geometry_msgs::Point32 position, const nav_msgs::Odometry& odometryData_old,const nav_msgs::Odometry& odometryData_new);
	geometry_msgs::Point transform_vel(geometry_msgs::Point position, const nav_msgs::Odometry& odometryData_old,const nav_msgs::Odometry& odometryData_new);

	ros::NodeHandle n_;
	nav_msgs::Odometry odometry_data_;
	nav_msgs::Odometry wheel_odometry_data;
	nav_msgs::OccupancyGrid map_data_;
	sensor_msgs::LaserScan scan_data_;
	sensor_msgs::LaserScan scan_data_old;
	geometry_msgs::Polygon shapes[MAX_OBJECTS];
	ros::Subscriber odometry_sub_;
	ros::Subscriber odometry_sub_2;
	ros::Subscriber map_sub_;
	ros::Subscriber scan_sub_;
	ros::Publisher object_pub_;
	ros::Publisher marker_pub_;
	ros::Publisher marker_Arrow_pub_;
	geometry_msgs::TransformStamped Lidar2base;
	geometry_msgs::TransformStamped odom2map;

	int seq;
	int tf_frame_listener_thread;



	
	//float xy_positions[SCAN_SIZE][2];
  	//int clusters[SCAN_SIZE];
	//int polygon[SCAN_SIZE];
	int polygon_size[MAX_OBJECTS];
	
	struct polygon_points{
		geometry_msgs::Point point;
		float range;
		float angle;
		int cluster; 
		int polygon_num;
	}typedef polygon_points;
	std::vector<polygon_points> polygon_point_list;
	
	double roll, pitch, yaw;
	double x,y,z;

	struct object_attributes{
		int sides_amount;
		float longest_size;
		float average_angle;
		float estimated_x;
		float estimated_y;
		geometry_msgs::Polygon points;
	}typedef object_attributes;
	object_attributes object_attributes_list[MAX_OBJECTS];

	

	int object_match[MAX_OBJECTS];
	//float object_match_ratio[MAX_OBJECTS][MAXTRACKS];

	//tracker multitracker;/*
	struct tracker_attributes{
		int confirmed;
		int age;
		int last_seen;
		int sides_amount;
		float longest_size;
		float average_angle;
		double time;
		float color[4];
		geometry_msgs::Polygon points;
		KalmanFilter tracker;
  	}typedef tracker_attributes;
//KalmanFilter tracker;
  	tracker_attributes trackers[MAXTRACKS];

	bool wheel_received = false;
	bool map_received = false;
	bool odom_received = false;
	bool scan_received = false;



	
};
#endif
