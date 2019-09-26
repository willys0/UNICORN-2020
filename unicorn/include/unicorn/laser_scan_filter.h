/**	
*	@file laser_scan_filter.h
*	@Author Alexander Karlsson (akn13013@student.mdh.se)
*/

#ifndef LASER_SCAN_FILTER_H
#define LASER_SCAN_FILTER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

/** @brief Class for the main laser filter node
*
*	Limits the angle of a laserscan topic "scan" and publishes
*	on topic based on rosparam "scan_topic".	
*/
class LaserFilter
{
public:
	/** @brief Laser class constructor.
	*	
	*	Initiates laserscan subscribing/advertising, reads rosparams,	
	*	and sets static pose of laser.
	*/
	LaserFilter();
	~LaserFilter();
	/** @brief Callback for laserscan topic
	*	
	*	Reads latest laserscan message, limits the angle, 
	*	and publishes a new message.
	*/
	void scanCallback(const sensor_msgs::LaserScan& input_scan);
	void scanCallback22(const sensor_msgs::LaserScan& input_scan);
	void publishScan();
	/** @brief Computes static pose of base laser.
	*	
	*	@param heading 	relative pose between rosparam "frame_id" and base_laser.
	*	@return 1 if pose was found, 0 otherwise.
	*/
	int getLaserPose(float& heading);
private:
	ros::NodeHandle n_;
	ros::Subscriber scan_sub_;
	ros::Publisher scan_pub_;
	sensor_msgs::LaserScan scan_;
	std::string chassis_frame_; /**< id of frame connected to odometry frame*/
	double lower_angle_;	/**< lower angular limit of the laserscan*/
	double upper_angle_;	/**< upper angular limit of the laserscan*/
	tf::TransformListener tf_listener_;
};

#endif
