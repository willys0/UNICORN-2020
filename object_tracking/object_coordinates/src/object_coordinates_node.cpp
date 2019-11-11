#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PointStamped.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <Eigen/Dense>
#include <Eigen/LU>

#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <string>

using namespace Eigen;

geometry_msgs::PointStamped msg;

void callback(const sensor_msgs::Image::ConstPtr& im_msg, const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb_msg)
{
	int xmin, xmax, ymin, ymax, width;
	float* depths;
	Vector3f coordinates;
	Matrix3f intrinsic;
	
	
	xmin = xmax = ymin = ymax = -1;
	width = 1280;
	depths = NULL;

	//filter out anything but humans
	if (bb_msg->bounding_boxes[0].id == 0) {
		//get the middle pixel in the bounding box
		//save the bounding box as integers
		int x = (bb_msg->bounding_boxes[0].xmin+bb_msg->bounding_boxes[0].xmax)/2;
		int y = (bb_msg->bounding_boxes[0].ymin+bb_msg->bounding_boxes[0].ymax)/2;
	
		// Get a pointer to the depth values casting the data
		depths = (float*)(&im_msg->data[0]);
	
		coordinates(0) = x;
		coordinates(1) = y;
		coordinates(2) = depths[x+y*im_msg->width];
	
		//to do: investigate
		//Intrinsic matrix to compensate for camera focal lengt, zoom and distortion
		intrinsic << 	678.2827758789062,	0,					647.6021728515625,
						0,					678.2827758789062,	370.912109375, 	
						0,					0,					1.0;


		//convert from pixel coordinates to world coordinates
		coordinates = intrinsic.inverse()*coordinates;
	
		//transcribe values to message
		msg.point.x = coordinates(0);
		msg.point.y = coordinates(1);
		msg.point.z = coordinates(2);
		msg.header.stamp = im_msg->header.stamp;
		msg.header.frame_id = im_msg->header.frame_id;
	}
}

/**
 * Node main function
 */
int main(int argc, char** argv) 
{

	//initialize
    ros::init(argc, argv, "tx2_object_coordinates");
    ros::NodeHandle n;
	//notify master of subscribers
	message_filters::Subscriber<sensor_msgs::Image> image_sub(n,"/zed/zed_node/depth/depth_registered",1);
	message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bb_sub(n,"/darknet_ros/bounding_boxes",1);
	//synchronization filter
	message_filters::TimeSynchronizer<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> sync(image_sub, bb_sub, 100);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	//notify master of publisher
	ros::Publisher pubVector= n.advertise<geometry_msgs::PointStamped>("tx2/object_coordinates/point",1);
	
	ros::Rate loop_rate(20);
	while (ros::ok()) {
		//publish message
		pubVector.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}

