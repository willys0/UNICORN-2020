#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"
//#include "movement_compensation_node.h"

#include <string>

#define RAD2DEG 57.295779513

//since I havent set up a .h file yet
tf2::Vector3 matrixvectorprod(tf2::Matrix3x3, tf2::Vector3);
tf2::Vector3 transformation(tf2::Matrix3x3, tf2::Vector3, tf2::Vector3);
tf2::Vector3 depth(int, int, int, int, int, int, float*, tf2::Matrix3x3);

// START OF CLASS MOVEMENTCOMPENSATION
class MovementCompensation {
	public:
		int xmin, xmax, ymin, ymax, width, height;
		float* depths;
		tf2::Vector3 translation;
		tf2::Matrix3x3 rotation, cameraMatrix;

		MovementCompensation();
		~MovementCompensation();

		// Callback functions
		void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
		void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
		void boundingboxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);
		//void cameramatrixCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

	private:

};

MovementCompensation::MovementCompensation()
{
	xmin = xmax = ymin = ymax = 0;
	depths = NULL;
}

MovementCompensation::~MovementCompensation()
{
}

void MovementCompensation::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{

    // Camera position in map frame
	translation.setX(msg->pose.position.x);
	translation.setY(msg->pose.position.y);
	translation.setZ(msg->pose.position.x);

    // Orientation quaternion
    tf2::Quaternion q(
	msg->pose.orientation.x,
	msg->pose.orientation.y,
	msg->pose.orientation.z,
	msg->pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    rotation.setRotation(q);
	

}

void MovementCompensation::depthCallback(const sensor_msgs::Image::ConstPtr& msg) 
{
	int i = 0;
	//get the height and width in pixels
	width = msg->width;
	height = msg->height;
	// Get a pointer to the depth values casting the data
	// pointer to floating point
	depths = (float*)(&msg->data[0]);

}
void MovementCompensation::boundingboxesCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	//save the bounding box as integers
	xmin = msg->bounding_boxes[0].xmin;
	ymin = msg->bounding_boxes[0].ymin;
	xmax = msg->bounding_boxes[0].xmax;
	ymax = msg->bounding_boxes[0].ymax;

}
//void MovementCompensation::cameramatrixCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
//{	
	//Save the camera matrix as a tf2::vector3x3 to allow the use of their functions
//	cameraMatrix.setValue(msg->K[0],msg->K[1],msg->K[2],msg->K[3],msg->K[4],msg->K[5],msg->K[6],msg->K[7],msg->K[8]);
	
//}


//END OF CLASS

tf2::Vector3 depth(int xmin, int xmax, int ymin, int ymax, int width, int height, float* depthmap, tf2::Matrix3x3 inverse) {
	tf2::Vector3 point, product;
	//out of bounds check
	if((depthmap == NULL) || xmin == -1) {
		product.setX(-1);
		product.setY(-1);
		product.setZ(-1);
		return product;
	}
	//get the point of interest
	int x = (xmin+xmax)/2;
	int y = (ymin+ymax)/2;
	//convert pixel position to meter
	point.setX(x);
	point.setY(y);
	point.setZ(1);
	product = matrixvectorprod(inverse, point);
	float dep = depthmap[x+width*y];
	product.setZ(dep);
	return product;
}

tf2::Vector3 ransformation(tf2::Matrix3x3 rotation, tf2::Vector3 translation, tf2::Vector3 point) {
	//point in reference to original camera position = rotation(point-translation)
	tf2::Vector3 subtracted, row1, row2, row3, result;
	subtracted.setX(point.getX()-translation.getX());
	subtracted.setY(point.getY()-translation.getY());
	subtracted.setZ(point.getZ()-translation.getZ());
	//becuase you cant multiply matrices with vectors... or transpose vectors...
	return matrixvectorprod(rotation, subtracted);
}

tf2::Vector3 matrixvectorprod(tf2::Matrix3x3 matrix, tf2::Vector3 vector) {
	//becuase tf2 doesnt support multiply matrices with vectors... or transpose vectors...
	tf2::Vector3 row1, row2, row3, product;
	row1 = matrix.getRow(0);
	row2 = matrix.getRow(1);
	row3 = matrix.getRow(2);
	product.setX(row1.getX()*vector.getX() + row1.getY()*vector.getY() + row1.getZ()*vector.getZ());
	product.setY(row2.getX()*vector.getX() + row2.getY()*vector.getY() + row2.getZ()*vector.getZ());
	product.setZ(row3.getX()*vector.getX() + row3.getY()*vector.getY() + row3.getZ()*vector.getZ());
	return product;

}
/**
 * Node main function
 */
int main(int argc, char** argv) {
	//initialize
    ros::init(argc, argv, "movement_compensation");
    ros::NodeHandle n;
	MovementCompensation nc;
	tf2::Vector3 point, vector;
	geometry_msgs::Point msg;

	//get the inverse of the intrinsic camera matrix(value taken from topic "zed/zed_node/rgb/camera_info")
	tf2::Matrix3x3 cameraMatrix;
	//remove last column to get it working without extrinsic parameters
	cameraMatrix.setValue(	678.2827758789062,	0,			647.6021728515625,
				0,			678.2827758789062,	370.912109375, 	
				0,			0,			1.0);
	tf2::Matrix3x3 inv;
	inv = cameraMatrix.inverse();
	//notify master of subscribers and publishers
    ros::Subscriber subPose	= n.subscribe("/zed/zed_node/pose", 1, &MovementCompensation::poseCallback, &nc);
    ros::Subscriber subDepth    = n.subscribe("/zed/zed_node/depth/depth_registered", 1, &MovementCompensation::depthCallback, &nc);
	ros::Subscriber subBb	= n.subscribe("/darknet_ros/bounding_boxes", 1, &MovementCompensation::boundingboxesCallback, &nc);
	//ros::Subscriber subCM	= n.subscribe("zed/zed_node/rgb/camera_info", 1, &MovementCompensation::cameramatrixCallback, &nc);
	ros::Publisher pubVector= n.advertise<geometry_msgs::Point>("/movement_compensation/position",1);


	//12-9 fps, ideally every node should be timed and this value tweaked
	ros::Rate loop_rate(10);
	while(ros::ok()){
		//get intrinsic camera matrix (3x4)
			//this value is hardcoded since it is static
		//get extrinsic camera matrix (4x4)
			
			//youtube.com/watch?v=fVJeJMWZcq8 15:48
		//multiply I*E
		//get the inverse of the result
		//multiply the pixel coordinates with the inverse
		
		//extract extrinsic parameters and multiply it with intrinsic


		//set vector depth to -1 to check if there is new data before transformation
		point.setZ(-1.0);
		//in case there is no new bounding box, dont bother calculating distance
		if(nc.xmin != -1){
			point = depth(nc.xmin, nc.xmax, nc.ymin, nc.ymax, nc.width, nc.height, nc.depths, inv);
		}
		//set boundingbox to -1 to check if new data has been read
		nc.xmin = nc.xmax = nc.ymin = nc.ymax = -1;
		//in case there is no new message
		if(point.getZ() != -1) {
			//vector = transformation(nc.rotation, nc.translation, vector);
			msg.x = point.getX();
			msg.y = point.getY();
			msg.z = point.getZ();

			pubVector.publish(msg);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
}
