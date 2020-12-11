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
    void shape_extraction_setvar(float *lambda_p,float *max_dist_laser_p,int *static_remove_dist_p,int* min_size_cluster_p, float *polygon_tolerance_p, int *polygon_min_points_p);

	
	//void adaptive_break_point(const nav_msgs::Odometry& odometryData, const sensor_msgs::LaserScan& scan, geometry_msgs::TransformStamped BaseLaser2BaseFrame);
	void adaptive_break_point(const sensor_msgs::LaserScan& scan);
	void static_map_filter(const nav_msgs::OccupancyGrid& map);
	void polygon_extraction();
	void polygon_attribute_extraction();


	struct clustered_point{
		geometry_msgs::Point point;
		float range;
		float angle;
	}typedef clustered_point;

	struct cluster{
		std::vector<clustered_point> cluster;
		int clusterNr;
		float max_range;
		float min_range;
	}typedef cluster;
	
	std::vector<cluster> cluster_list;


	struct object_attributes{
		int sides_amount;
		float longest_size;
		float average_angle;
		geometry_msgs::Point position;
		std::vector<clustered_point> cluster;
		geometry_msgs::Polygon polygon;
	}typedef object_attributes;
	std::vector<object_attributes> object_attributes_list;


	int *min_size_cluster;
	float *max_dist_laser;
	float *lambda;
	int *static_remove_dist;
	int *polygon_min_points;
	float *polygon_tolerance;

private:
	void extract_corners(int startpoint,int endpoint, int length,int clusterNr, std::vector<int> *zeroVector);
	void search_longest(int startpoint, int current_point,int end_point, int length, float distance_S, int itteration, int max_itteration, int *best_point, float *best_dist, int clusterNr);

	



};

#endif