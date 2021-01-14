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
#include <Eigen/Dense>
 
#include "kalman/kalman.hpp"
#include "Hungarian/Hungarian.h"
#include "shape_extraction.hpp"
//#include "ICP/icpPointToPlane.h"

#define MAX_OBJECTS 100

class association
{
public:
    association();

	void association_setvar(int *CONFIRMED_TRACK_p, int *TRACKER_LIFE_p,float *max_similarty_deviation_p, float *sim_adj_dist_p,float *sim_adj_angle_p,float *sim_adj_side_p,float *sim_adj_xpos_p,float *sim_adj_ypos_p,float *sim_adj_posdiff_p);
	void associate(const std::vector<shape_extraction::object_attributes>& object_attributes_list,const nav_msgs::Odometry& odometryData,const geometry_msgs::TransformStamped BaseLaser2BaseFrame, ros::Time stamp);
	void estimate_new_position(double time);
	void update_position();
	//void estimate_moment(int trackerid, const shape_extraction::object_attributes& object, double dt);

	//tracker multitracker;/*
	struct tracker_attributes{
		int confirmed;
		int age;
		int last_seen;
		int sides_amount;
		float length;
		float width;
		float average_angle;
		double time;
		float color[4];
		geometry_msgs::Polygon points;
		std::vector<geometry_msgs::Point> trace; // Position estimation over time for an object
		KalmanFilter tracker;
  	}typedef tracker_attributes;

	std::vector<tracker_attributes> trackers;
	

	float *sim_adj_dist;
	float *sim_adj_angle;
	float *sim_adj_side;
	float *sim_adj_xpos;
	float *sim_adj_ypos;
	float *sim_adj_posdiff;
	float *max_similarty_deviation;
	int *CONFIRMED_TRACK;
	int *TRACKER_LIFE;

	float sim_adj_dist_std = 1.0;
	float sim_adj_angle_std = 1.0;
	float sim_adj_side_std = 1.0;
	float sim_adj_xpos_std = 5.2;
	float sim_adj_ypos_std = 5.2;
	float sim_adj_posdiff_std = 2.2;
	float max_similarty_deviation_std = 1.5;
	int CONFIRMED_TRACK_std = 50;
	int TRACKER_LIFE_std = 200;


private:
	//void association::transform_polygon(geometry_msgs::Polygon polygon_in, geometry_msgs::Polygon polygon_out, const nav_msgs::Odometry& odometryData,geometry_msgs::TransformStamped BaseLaser2BaseFrame);
	geometry_msgs::Point transform_point(geometry_msgs::Point position, const nav_msgs::Odometry& odometryData,geometry_msgs::TransformStamped BaseLaser2BaseFrame);
	void calculateVel(shape_extraction::object_attributes object, int trackernr,float *sum);
	void initiate_Trackers(shape_extraction::object_attributes object,double time, const nav_msgs::Odometry& odometryData,geometry_msgs::TransformStamped BaseLaser2BaseFrame);
	void update_tracker(shape_extraction::object_attributes object,int trackerID, double dt);

};

#endif