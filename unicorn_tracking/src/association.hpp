#ifndef ASSOCIATION_H
#define ASSOCIATION_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Polygon.h>

#include <stdio.h>      
#include <time.h>    
#include <iostream>  
 
#include "kalman/kalman.hpp"
#include "Hungarian/Hungarian.h"
#include "shape_extraction.hpp"


#define MAX_OBJECTS 100

class association
{
public:
    association();

	/*
	struct object_attributes{
		int sides_amount;
		float longest_size;
		float average_angle;
		float estimated_x;
		float estimated_y;
		geometry_msgs::Polygon polygon;
	}typedef object_attributes;*/

	void associate(const std::vector<shape_extraction::object_attributes>& object_attributes_list,const nav_msgs::Odometry& odometryData,const geometry_msgs::TransformStamped BaseLaser2BaseFrame, ros::Time stamp);
	
	void estimate_new_position(double time);
	void update_position();

	void calculateOdometryChange(const nav_msgs::Odometry& odometryData_new);
	
	//void estimate_odometry(const sensor_msgs::LaserScan& scan_new, const sensor_msgs::LaserScan& scan_old);

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
		geometry_msgs::Polygon cluster;
		geometry_msgs::Polygon points;
		KalmanFilter tracker;
  	}typedef tracker_attributes;

	std::vector<tracker_attributes> trackers;
	

	float sim_adj_dist = 1.0;
	float sim_adj_angle = 1.0;
	float sim_adj_side = 1.0;
	float sim_adj_xpos = 2.2;
	float sim_adj_ypos = 2.2;
	float sim_adj_posdiff = 2.2;
	float max_similarty_deviation = 1.5;

	int CONFIRMED_TRACK = 50;
	int TRACKER_LIFE = 200;


private:
	//void association::transform_polygon(geometry_msgs::Polygon polygon_in, geometry_msgs::Polygon polygon_out, const nav_msgs::Odometry& odometryData,geometry_msgs::TransformStamped BaseLaser2BaseFrame);
	geometry_msgs::Point transform_point(geometry_msgs::Point position, const nav_msgs::Odometry& odometryData,geometry_msgs::TransformStamped BaseLaser2BaseFrame);
	void calculateVel(shape_extraction::object_attributes object, int trackernr,float *sum, const nav_msgs::Odometry& odometryData,const geometry_msgs::TransformStamped BaseLaser2BaseFrame);
	void initiate_Trackers(shape_extraction::object_attributes object,double time, const nav_msgs::Odometry& odometryData,geometry_msgs::TransformStamped BaseLaser2BaseFrame);

	bool OdometryChange_initiated = false;
	nav_msgs::Odometry odometryData;
	double odom_change_x = 0;
	double odom_change_y = 0;
	double yaw_change = 0;
};

#endif