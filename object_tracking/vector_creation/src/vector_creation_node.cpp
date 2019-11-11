#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include "vector_creation/vectors.h"
#include "vector_creation/vector.h"


// START OF CLASS MOVEMENTCOMPENSATION
class VectorCreation {
  public:
	int listsize;

	VectorCreation() : listsize()
	{
	listsize = 4;
		for (int i = 0; i > listsize; i++) {
			x[i] = 1;
			y[i] = 1;
			z[i] = 1;
			time[i] = 1;
		}
		
	}
	vector_creation::vector message;
	// Callback functions
	void positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
  private:
	float x[4], y[4], z[4], time[4];
	float xvel, yvel, zvel;
	
};

void VectorCreation::positionCallback(const geometry_msgs::PointStamped::ConstPtr& msg) 
{
	//save 5 latest points in arrays
	for(int i = 0; i < listsize-1; i++) {
		x[i] = x[i+1];
		y[i] = y[i+1];
		z[i] = z[i+1];
		time[i] = time[i+1];
	}
	x[0] = msg->point.x;
	y[0] = msg->point.y;
	z[0] = msg->point.z;
	time[0] = msg->header.stamp.nsec;

	xvel = 0;
	yvel = 0;
	zvel = 0;

	//calculate and add average velocities between all points
	for(int i = 0; i < listsize-1; i++) {
		xvel += (x[i]-x[i+1])/1000000000*(time[i]-time[i+1]);
		yvel += (y[i]-y[i+1])/1000000000*(time[i]-time[i+1]);
		zvel += (z[i]-z[i+1])/1000000000*(time[i]-time[i+1]);//add 3 zeros
		//ROS_INFO("x: %f   y: %f   z: %f",xvel, yvel, zvel);
	}
	//fill message with information
	message.xvelocity = xvel/listsize;
	message.yvelocity = yvel/listsize;
	message.zvelocity = zvel/listsize;
	message.xposition = x[0];
	message.yposition = y[0];
	message.zposition = z[0];
	
}

/**
 * Node main function
 */
int main(int argc, char** argv) {

    ros::init(argc, argv, "tx2_vector_creation");
    ros::NodeHandle n;
	VectorCreation vc;
	vector_creation::vector msg;

	ros::Subscriber subPosition    	= n.subscribe("tx2/tf_filter/point", 1, &VectorCreation::positionCallback, &vc);
	ros::Publisher pubVector	= n.advertise<vector_creation::vector>("tx2/vector_creation/vector",1);

	ros::Rate loop_rate(20);
	while(ros::ok()){
		
		pubVector.publish(vc.message);
		ros::spinOnce();
		loop_rate.sleep();
	}
   	return 0;
}



/*
if(position_vector.size() > 2) {
	for (int i=0; i<(position_vector.size()-1); i++) {
		double total_time;
				
		if (position_vector[i+1].header.stamp.sec == position_vector[i].header.stamp.sec)
			total_time = 0.000000001*(position_vector[i+1].header.stamp.nsec - position_vector[i].header.stamp.nsec);
		else if (position_vector[i+1].header.stamp.sec == (position_vector[i].header.stamp.sec + 1))
			total_time = 0.000000001*(position_vector[i+1].header.stamp.nsec + (1000000000-position_vector[i].header.stamp.nsec));
		else 				
			std::cout << "Calculating obstacle vector took too long" << std::endl;
				
		msg.xvelocity += (position_vector[i+1].point.x - position_vector[i].point.x) / total_time;
		msg.yvelocity += (position_vector[i+1].point.y - position_vector[i].point.y) / total_time;
		msg.zvelocity += (position_vector[i+1].point.z - position_vector[i].point.z) / total_time;
				
	}
	msg.xvelocity /= (position_vector.size()-1);
	msg.yvelocity /= (position_vector.size()-1);
	msg.zvelocity /= (position_vector.size()-1);

	
	msg.xposition = position_vector[position_vector.size()-1].point.x;
	msg.yposition = position_vector[position_vector.size()-1].point.y;
	msg.zposition = position_vector[position_vector.size()-1].point.z;

}



*/




