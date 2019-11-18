#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include "vector_creation/vectors.h"
#include "vector_creation/vector.h"

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <gazebo/msgs/link.pb.h>
#include <gazebo/msgs/link_data.pb.h>

#include <vector>
#include <queue>
#include <iostream>
#include "std_msgs/String.h"
#include <sstream>




std::vector<geometry_msgs::PointStamped> position_vector;


// START OF CLASS MOVEMENTCOMPENSATION
class VectorCreation {
	public:
		float x, y, z;

		// Callback functions
		//void positionCallback(const geometry_msgs::Point::ConstPtr& msg);
		void positionCallback(ConstPosesStampedPtr &posesStamped);


	private:

};


/*void VectorCreation::positionCallback(const geometry_msgs::Point::ConstPtr& msg) 
{
	x = msg->x;
	y = msg->y;
	z = msg->z;

}*/

void positionCallback(ConstPosesStampedPtr &posesStamped)
{
	// Dump the message contents to stdout.
	//std::cout << "Tjena \n";
	//std::cout << posesStamped->DebugString();

	::google::protobuf::int32 sec = posesStamped->time().sec();
	::google::protobuf::int32 nsec = posesStamped->time().nsec();
	//std::cout << "Read time: sec: " << sec << " nsec: " << nsec << std::endl;
	geometry_msgs::PointStamped position_point;

	for (int i =0; i < posesStamped->pose_size(); ++i)
	{
		const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
		std::string name = pose.name();
		if (name == std::string("Construction Barrel_moving"))
		{
			const ::gazebo::msgs::Vector3d &position = pose.position();

			double x = position.x();
			double y = position.y();
			double z = position.z();
			position_point.point.x = x;
			position_point.point.y = y;
			position_point.point.z = z;
			position_point.header.stamp.nsec = nsec;
			position_point.header.stamp.sec = sec;


		//std::cout << "Read position: x: " << x << " y: " << y << " z: " << z << std::endl;
		}
	}

	if(position_vector.size() >= 5)
		position_vector.erase(position_vector.begin());

  	position_vector.push_back(position_point);

	//std::cout << "Position vector size: " << position_vector.size() << std::endl;
	//std::cout << "Position vector: " << std::endl;
	//for (int i=0; i < position_vector.size(); i++)
	//	std::cout << i << " Time: " << position_vector[i].header.stamp.sec << " " << position_vector[i].header.stamp.nsec << " Position: " << position_vector[i].point.x << " " << position_vector[i].point.y << " " << position_vector[i].point.z << std::endl;
}

//end of class


/**
 * Node main function
 */
int main(int argc, char** argv) {

	  // Load gazebo
  gazebo::client::setup(argc, argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

    ros::init(argc, argv, "vector_creation");
	//std::cout << "Vector creation created" << std::endl;
    ros::NodeHandle n;
	VectorCreation nc;
	vector_creation::vector msg;
	//std::vector<vector_creation::vector> msg_to_send;
	position_vector.clear();

	//ros::Subscriber subPosition    	= n.subscribe("movement_compensation/position", 1, &VectorCreation::positionCallback, &nc);
	
	//ros::Subscriber subPosition    	= n.subscribe("~/pose/info", 1, &VectorCreation::positionCallback, &nc);
	ros::Publisher pubVector	= n.advertise<vector_creation::vector>("/vector_generation/vector",1);
 	//ros::Publisher pubVector	= n.advertise<std_msgs::String>("chatter",1000);
	gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", positionCallback);

	ros::Rate loop_rate(5);
	//int count = 0;
	while(ros::ok()){
		/*
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str());
		*/

		//gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", positionCallback);
		//gazebo::common::Time::MSleep(10);
		

		

		/*msg.xposition = 0;
		msg.yposition = 0;
		msg.zposition = 0;
		msg.xvelocity = 0;
		msg.yvelocity = 0;
		msg.zvelocity = 0;*/

		//create_vector()
		std::cout << "Vector creation loop" << std::endl;
		if(position_vector.size() > 2)
		{
			//::google::protobuf::int32 total_time;

			for (int i=0; i<(position_vector.size()-1); i++)
			{
				double total_time;
				//total_time = (position_vector[i+1].header.stamp.sec - position_vector[i].header.stamp.sec) + 0.000000001*(position_vector[i+1].header.stamp.nsec - position_vector[i].header.stamp.nsec);
				
				if (position_vector[i+1].header.stamp.sec == position_vector[i].header.stamp.sec)
					total_time = 0.000000001*(position_vector[i+1].header.stamp.nsec - position_vector[i].header.stamp.nsec);
				else if (position_vector[i+1].header.stamp.sec == (position_vector[i].header.stamp.sec + 1))
					total_time = 0.000000001*(position_vector[i+1].header.stamp.nsec + (1000000000-position_vector[i].header.stamp.nsec));
				else 				
					std::cout << "Calculating obstacle vector took too long" << std::endl;
				

				msg.xvelocity += (position_vector[i+1].point.x - position_vector[i].point.x) / total_time;//(position_vector[i+1].header.stamp.nsec - position_vector[i].header.stamp.nsec);
				msg.yvelocity += (position_vector[i+1].point.y - position_vector[i].point.y) / total_time;//(position_vector[i+1].header.stamp.nsec - position_vector[i].header.stamp.nsec);
				msg.zvelocity += (position_vector[i+1].point.z - position_vector[i].point.z) / total_time;//(position_vector[i+1].header.stamp.nsec - position_vector[i].header.stamp.nsec);
				
				/*std::cout << "Time sec: " << position_vector[i+1].header.stamp.sec << " - " << position_vector[i].header.stamp.sec << std::endl;
				std::cout << "Time nsec: " << position_vector[i+1].header.stamp.nsec << " - " << position_vector[i].header.stamp.nsec << std::endl;
				std::cout << "Time difference: " << total_time << std::endl;
				*/
			}
			msg.xvelocity /= (position_vector.size()-1);
			msg.yvelocity /= (position_vector.size()-1);
			msg.zvelocity /= (position_vector.size()-1);


			//msg.xvelocity *= (position_vector[position_vector.size()-1].header.stamp.nsec - position_vector[0].header.stamp.nsec);

			msg.xposition = position_vector[position_vector.size()-1].point.x;
			msg.yposition = position_vector[position_vector.size()-1].point.y;
			msg.zposition = position_vector[position_vector.size()-1].point.z;

			//std::cout << "Position: " << msg.xposition << " " << msg.yposition << " " << msg.zposition << " Velocity: " << msg.xvelocity << " " << msg.yvelocity << " " << msg.zvelocity << std::endl;
			std::cout << "Message sent where position x = " << msg.xposition << std::endl;;
			pubVector.publish(msg);
		}
		
	

		/*msg.xposition = nc.x;
		msg.yposition = nc.y;
		msg.zposition = nc.z;
		msg.xvelocity = 1.0;
		msg.yvelocity = 1.0;
		msg.zvelocity = 1.0;
		
		pubVector.publish(msg);*/
		//pubVector.publish(msg);
		//gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", positionCallback);
		//gazebo::common::Time::MSleep(10);
		

		ros::spinOnce();
		loop_rate.sleep();
		//++count;
	}
   	return 0;
}
