#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"
#include "movement_compensation.h"

#include <string>

#define RAD2DEG 57.295779513


// START OF CLASS MOVEMENTCOMPENSATION
class MovementCompensation {
	public:
		int xmin, xmax, ymin, ymax, width, height;
		double tx, ty, tz;
		float* depths;
		tf2::Matrix3x3 m;
		

		MovementCompensation();
		~MovementCompensation();

		// Callback functions
		void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
		void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void boundingboxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);

	private:

};

MovementCompensation::MovementCompensation()
{
	xmin = xmax = ymin = ymax = 0;
	tx = ty = tz = 0;
	depths = NULL;
}

MovementCompensation::~MovementCompensation()
{
}

void MovementCompensation::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{

    // Camera position in map frame
    tx = msg->pose.position.x;
    ty = msg->pose.position.y;
    tz = msg->pose.position.z;

    // Orientation quaternion
    tf2::Quaternion q(
	msg->pose.orientation.x,
	msg->pose.orientation.y,
	msg->pose.orientation.z,
	msg->pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    m.setRotation(q);
	

}

void MovementCompensation::depthCallback(const sensor_msgs::Image::ConstPtr& msg) 
{
	int i = 0;
	width = msg->width;
	// Get a pointer to the depth values casting the data
	// pointer to floating point
	depths = (float*)(&msg->data[0]);

}
void MovementCompensation::boundingboxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{

	//save the bounding box
	xmin = msg->bounding_boxes[0].xmin;
	ymin = msg->bounding_boxes[0].ymin;
	xmax = msg->bounding_boxes[0].xmax;
	ymax = msg->bounding_boxes[0].ymax;

}

struct vector {
float x;
float y;
float z;
};
