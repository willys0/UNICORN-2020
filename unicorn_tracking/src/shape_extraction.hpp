#ifndef SHAPE_EXTRACTION_H
#define SHAPE_EXTRACTION_H


#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Polygon.h>

class shape_extraction
{
public:
	shape_extraction();

    void shape_extraction_setvar(float *lambda_p,float *max_dist_laser_p,int *static_remove_dist_p,float* static_remove_ratio_p,int* min_size_cluster_p, float *polygon_tolerance_p, int *polygon_min_points_p);

	
	void adaptive_break_point(const sensor_msgs::LaserScan& scan);
	void static_map_filter(const nav_msgs::OccupancyGrid& map,const geometry_msgs::TransformStamped BaseFrame2Odomframe,const geometry_msgs::TransformStamped BaseLaser2BaseFrame,const geometry_msgs::TransformStamped odom2map);
	void transformObjects(const nav_msgs::Odometry& odometryData,const geometry_msgs::TransformStamped BaseLaser2BaseFrame);
	void polygon_extraction();
	void polygon_attribute_extraction();


	struct cluster{
		std::vector<geometry_msgs::Point> cluster;
		int clusterNr;
		float max_range;
		float min_range;
	}typedef cluster;
	std::vector<cluster> cluster_list;


	struct object_attributes{
		int sides_amount;
		float length;
		float width;
		float average_angle;
		std::vector<geometry_msgs::Point> cluster;
		geometry_msgs::Point position;
		geometry_msgs::Polygon polygon;
	}typedef object_attributes;
	std::vector<object_attributes> object_attributes_list;



	int min_size_cluster_std = 4;
	float max_dist_laser_std = 10;
	float lambda_std = 0.15;
	int static_remove_dist_std = 5;
	float static_remove_ratio_std = 0.5;
	int polygon_min_points_std = 3;
	float polygon_tolerance_std = 1.04;

	int *min_size_cluster;
	float *max_dist_laser;
	float *lambda;
	int *static_remove_dist;
	float *static_remove_ratio;
	int *polygon_min_points;
	float *polygon_tolerance;

private:
	void extract_corners(int startpoint,int endpoint, int length,int clusterNr, std::vector<int> *zeroVector);
	void search_longest(int startpoint, int current_point,int end_point, int length, float distance_S, int itteration, int max_itteration, int *best_point, float *best_dist, int clusterNr);

	geometry_msgs::Point transform_point(geometry_msgs::Point position, const nav_msgs::Odometry& odometryData,geometry_msgs::TransformStamped BaseLaser2BaseFrame);



};

#endif