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
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <string>
#include <array>
using namespace Eigen;
geometry_msgs::PointStamped msg;
void callback(const sensor_msgs::Image::ConstPtr& im_msg, const darknet_ros_msgs::BoundingBoxes::ConstPtr& bb_msg)
{
//	ROS_INFO("object_coor callback called");
	int xmin, xmax, ymin, ymax, width;
	float* depths;
	Vector3f coordinates;
	Matrix3f intrinsic;
	
	
	xmin = xmax = ymin = ymax = -1;
	width = 672;
	depths = NULL;
	int size = 0;
	
	coordinates(2) = 10;
	//look for closest human with for loop
	for(int i = 0; i < bb_msg->bounding_boxes.size(); i++) {
		//filter out anything but humans
		if (bb_msg->bounding_boxes[i].id == 0) {
			//get the middle pixel in the bounding box
			//save the bounding box as integers
			int x = (bb_msg->bounding_boxes[i].xmin+bb_msg->bounding_boxes[i].xmax)/2;
			int y = (bb_msg->bounding_boxes[i].ymin+bb_msg->bounding_boxes[i].ymax)/2;
			// Get a pointer to the depth values casting the data
			depths = (float*)(&im_msg->data[i]);
			//Intrinsic matrix to compensate for camera focal lengt, zoom and distortion
			intrinsic << 	678.2827758789062,	0,					647.6021728515625,
							0,					678.2827758789062,	370.912109375, 	
							0,					0,					1.0;
			
			ROS_INFO("object depth: %f",depths[x+y*im_msg->width]);			
			coordinates(2) = depths[x+y*im_msg->width];
			//save the closest human
			if(coordinates(2) > depths[x+y*im_msg->width]){
				coordinates(0) = x;
				coordinates(1) = y;
				coordinates(2) = depths[x+y*im_msg->width];
			}
		}
	}
	//if human was found
	if(coordinates(2) != 10) {			 
		//convert from pixel coordinates to world coordinates
		coordinates = intrinsic.inverse()*coordinates;
		//transcribe values to message
		msg.point.x = coordinates(0);
		msg.point.y = coordinates(1);
		msg.point.z = coordinates(2);
		msg.header.stamp = im_msg->header.stamp;
		msg.header.frame_id = im_msg->header.frame_id;
		ROS_INFO("print obj coordinates");
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
	message_filters::Subscriber<sensor_msgs::Image> image_sub(n,"/zed/depth/depth_registered",10);
	message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bb_sub(n,"/darknet_ros/bounding_boxes",10);
	//synchronization filter
	//message_filters::TimeSynchronizer<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> sync(image_sub, bb_sub, 200);
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> SyncPolicy;
	
	message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), image_sub, bb_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	//notify master of publisher
	ros::Publisher pubVector= n.advertise<geometry_msgs::PointStamped>("/TX2/object_coordinates/point",1);
	
	ros::Rate loop_rate(20);
	while (ros::ok()) {
		//publish message
		pubVector.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}

