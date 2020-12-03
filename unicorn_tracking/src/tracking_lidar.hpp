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
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <unicorn_tracking/TrackingConfig.h>

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
#include "kalman.hpp"
#include "Hungarian.h"

#define PI 3.14159265
#define MAX_OBJECTS 100
#define SCAN_SIZE 800
#define MAXTRACKS MAX_OBJECTS
#define TRACKER_LIFE 1000
#define CONFIRMED_TRACK 100


class tracking_lidar
{
public:
	tracking_lidar();
	void odomCallback(const nav_msgs::Odometry& odometry);
	void mapCallback(const nav_msgs::OccupancyGrid& map);
	void scanCallback(const sensor_msgs::LaserScan& scan);
	void publishmsg();
	void adaptive_breaK_point();
	void static_map_filter();
	void polygon_extraction();
	void polygon_attribute_extraction();
	void object_publisher();
	void association();

	/* Variables that can be changed */
	float lambda;
	int max_dist_laser;
	int static_remove_dist;
	float polygon_tolerance;
	int polygon_min_points;
	float min_twist_detection, max_similarty_deviation;
	float sim_adj_dist, sim_adj_angle, sim_adj_side, sim_adj_xpos, sim_adj_ypos;
	bool static_filter;
	std::string mapframeid = "/map";

private:
	void extract_corners(int startpoint,int endpoint, int length,int shape_nr);
	void search_longest(int startpoint, int current_point,int end_point, int length, float distance_S, int itteration, int max_itteration, int *best_point, float *best_dist);
	void initiate_Trackers();
	void estimate_new_position();
	void update_position();

	ros::NodeHandle n_;
	nav_msgs::Odometry odometry_data_;
	nav_msgs::OccupancyGrid map_data_;
	sensor_msgs::LaserScan scan_data_;
	geometry_msgs::Polygon shapes[MAX_OBJECTS];
	ros::Subscriber odometry_sub_;
	ros::Subscriber map_sub_;
	ros::Subscriber scan_sub_;
	ros::Publisher object_pub_;
	ros::Publisher marker_pub_;
	ros::Publisher marker_Arrow_pub_;

	int seq;

	float xy_positions[SCAN_SIZE][2];
	float xy_map_positions[SCAN_SIZE][2];
  	int clusters[SCAN_SIZE];
	int polygon[SCAN_SIZE];
	int polygon_size[MAX_OBJECTS];
	
	  
	double roll, pitch, yaw;
	double x,y,z;
	uint32_t mapx,mapy;

	struct object_attributes{
		int sides_amount;
		float longest_size;
		float average_angle;
		float estimated_x;
		float estimated_y;
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



	
};
#endif
