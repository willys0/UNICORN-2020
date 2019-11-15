#include <pluginlib/class_list_macros.h>
#include "local_planner.h"

//#define INT_MAX 2000


 //register this planner as a BaseLocalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(local_planner::LocalPlanner, nav_core::BaseLocalPlanner)

 using namespace std;

 //Default Constructor
 namespace local_planner {

LocalPlanner::LocalPlanner (){

}

LocalPlanner::LocalPlanner(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros){
   initialize(name, tf, costmap_ros);
}

/*
Function that returns the closest point on the path which is closest to the current robot position
Returns: Coordinates in world of the closest point on path
Arguments:
robot_pose: Pose of the robot
plan_index: Pointer that stores the index of the path
distance_to_path: Pointer that stores the actual distance to path
*/
geometry_msgs::Point LocalPlanner::closestPointOnPath(tf::Stamped<tf::Pose> robot_pose, int *plan_index, float *distance_to_path)//geometry_msgs::Point robot_loc)
{
    geometry_msgs::Point closest_point;
    float dx,dy,dist,shortest_dist;
	ros::Time now = ros::Time::now();

	tf::Stamped<tf::Pose> robot_world_pose;
	
	tf_.waitForTransform("/map", robot_base_frame_, now, ros::Duration(0.2));
	tf_.transformPose("/map", robot_pose, robot_world_pose);
 
	//std::cout << "Robot position... " << "Robot frame: " << robot_pose.getOrigin().getX() << " " << robot_pose.getOrigin().getY() << " Map frame: " << robot_world_pose.getOrigin().getX() << " " << robot_world_pose.getOrigin().getY() << endl;

	for(int i = 0; i < global_plan_temp.size(); i++) 
	{
		dx =  global_plan_temp[i].pose.position.x-robot_world_pose.getOrigin().getX();
		dy =  global_plan_temp[i].pose.position.y-robot_world_pose.getOrigin().getY();
		//std::cout << "Robot position: " << robot_world_pose.getOrigin().getX() << " " << robot_world_pose.getOrigin().getY();
		//std::cout << " Global plan position: " << global_plan_temp[i].pose.position.x << " " << global_plan_temp[i].pose.position.y << endl;
		dist = std::sqrt(dx*dx + dy*dy);
		//std::cout << "Iterator it = " << i << endl;
		if(i==0 || dist <= shortest_dist)
		{
			//std::cout << "TransformGlobalPlan 0: " << global_plan_temp[i].pose.position.x << " " << global_plan_temp[i].pose.position.y << endl;
			closest_point.x = global_plan_temp[i].pose.position.x;
			closest_point.y = global_plan_temp[i].pose.position.y;
			closest_point.z = 0;
			shortest_dist = dist; 
			*distance_to_path = dist; // Store the shortest distance
			*plan_index = i; // Store what index in the path that is closest
		}
	}
    return closest_point;
}

/*
Function that returns the heading of the path at the closest distance to the robot.
Returns: A unit vector
Arguments:
plan_index: The index of the path to start looking at
*/
tf::Vector3 LocalPlanner::unitVectorOfPath(int plan_index)
{
	geometry_msgs::PoseStamped second_point, first_point;
	tf::Vector3 vector_heading;
    float x_value, y_value;
    pair<float,float> vector_value;
	//std::cout << "Global_plan_temp size: " << global_plan_temp.size() << " Plan index: " << plan_index << endl;
    first_point = global_plan_temp[plan_index];
	//std::cout << "unit_vector1" << endl;
    if(global_plan_temp.size() == (plan_index + 1))
	{
		vector_heading.setValue(0, 0, 0);
	}
	else if(global_plan_temp.size() <= (plan_index + 5))
	{
        second_point = global_plan_temp[plan_index+1];
		x_value = second_point.pose.position.x - first_point.pose.position.x;
		y_value = second_point.pose.position.y - first_point.pose.position.y;
		vector_heading.setValue(x_value, y_value, 0);
		vector_heading.normalize();
	}
    else
	{
        second_point = global_plan_temp[plan_index+5];
		x_value = second_point.pose.position.x - first_point.pose.position.x;
		y_value = second_point.pose.position.y - first_point.pose.position.y;
		vector_heading.setValue(x_value, y_value, 0);
		vector_heading.normalize();
	}
	return vector_heading;
}

/*
Function that calculates the vector part of the attractive field part
Arguments:
ai (Starting point of a line segment)
robot_pose (The robot's pose)
unit_vector_ni (Vector from the start of a line segment towards the next point)
attractive_vector (Vector to return via a pointer)
*/
void LocalPlanner::fAttractiveVector(geometry_msgs::Point fa_point_path, tf::Stamped<tf::Pose> robot_pose, tf::Vector3 unit_vector_ni, tf::Vector3 *attractive_vector)
{
	tf::Stamped<tf::Pose> robot_world_pose;
	tf_.transformPose("/map", robot_pose, robot_world_pose);
	//tf_.waitForTransform("/map", robot_pose, robot_world_pose);

    tf::Vector3 robot_position(robot_world_pose.getOrigin().getX(), robot_world_pose.getOrigin().getY(), 0);
	//std::cout << "Robot pose : " << robot_world_pose.getOrigin().getX() << " " << robot_world_pose.getOrigin().getY() << endl;
    tf::Vector3 ai(fa_point_path.x ,fa_point_path.y, 0);
	//std::cout << "ai : " << ai.m_floats[0] << " " << ai.m_floats[1] << " " << ai.m_floats[2] << endl;
    tf::Vector3 ai_p = ai.operator-=(robot_position);
	//std::cout << "ai-p : " << ai_p.m_floats[0] << " " << ai_p.m_floats[1] << " " << ai_p.m_floats[2] << endl;
    tfScalar attractive_scalar = ai_p.dot(unit_vector_ni);
	//std::cout << "attractive scalar : " << attractive_scalar << endl;
    *attractive_vector = ai_p.operator-=(unit_vector_ni.operator*=(attractive_scalar));
	//std::cout << "Attractive middle part: " << attractive_vector->m_floats[0] << " " << attractive_vector->m_floats[1] << " " << attractive_vector->m_floats[2] << endl;
    attractive_vector->normalize();
	//std::cout << "Attractive middle part normalized: " << attractive_vector->m_floats[0] << " " << attractive_vector->m_floats[1] << " " << attractive_vector->m_floats[2] << endl;
    if(isnan(attractive_vector->m_floats[0]))
    {
        attractive_vector->setValue(0.0,0.0,0.0);
    }
}

/*
Function that calculates the repulsive vector of an static obstacle
*/
void LocalPlanner::repulsiveForce(tf::Stamped<tf::Pose> robot_pose, tf::Vector3 *repulsive_vector)
{
	// Robot size has a radius of 0.2m in simulation with center at middle.
	// In the paper, dmax = 1.25, radius = 0.5m, resolution = 0.1
	// dmax = 1 + radius / 2
	// If radius = 0.2, gives dmax = 1+0.2/2 = 1.1;
	// Every 
    int xt, yt, cost, force_x, force_y, deg;
    int scale = 1; // default 100
    int gain = 106; // 71 = 1meter // 2000 // Default 10000
    float dmax = 21; // default 1.25 // 1.1
	float repulsive_force, x_force, y_force, z_force;

	makeRepulsiveField(scale, gain, dmax, robot_pose.getOrigin().getX(), robot_pose.getOrigin().getY(), &repulsive_force, &deg);
    
	if (repulsive_force > 0)
	{
		y_force = repulsive_force*sin(deg*PI/180);
		x_force = repulsive_force*cos(deg*PI/180);
		z_force = 0;
		repulsive_vector->setValue(x_force,y_force,z_force);
	}
	else // If there is no repulsive force affecting the robot
	{
		repulsive_vector->setValue(0.0,0.0,0.0);
	}
}

/*
Function that calculates the repulsive force affecting the robot
*/
void LocalPlanner::makeRepulsiveField(int scale, int gain, float dmax, float pos_x, float pos_y, float *repulsive_force, int *deg)
{
    float d, d0, d2;
    d0 = 1.0/dmax; 
	float distance_to_obstacle;
	findClosestObjectEuclidean(deg, &distance_to_obstacle);
	d = distance_to_obstacle * costmap_->getResolution() * 10; // Convert distance to decimeters
	if (d < 3)
		d = 3;
    d2 = d / scale + 1;
    d = (1 / d2 - d0);
    d = d*d;

	if (d2 <= dmax) // If the robot is close enough to an obstacle to make that obstacle influence with the flow field
		*repulsive_force = gain*d;
	else
		*repulsive_force = 0;
}

/*
Function that looks in the local costmap to provide the distance and degree to the closest obstacle
*/
void LocalPlanner::findClosestObjectEuclidean(int *deg, float *distance_to_obstacle)
{
    float min_dist = 2000.0;
    float current_dist = 0;

    int current_point = 0;

    int min_x = 0;
    int min_y = 0;

    int dist_x = 0;
    int dist_y = 0;	

	unsigned int coordx = 0,coordy = 0;
	int deg_moving_obstacle = 100;

	int pos_x = costmap_->getSizeInCellsX() / 2; // Center of the costmap (Robot's position)
	int pos_y = costmap_->getSizeInCellsY() / 2;
	
	for (int h = 0; h < costmap_->getSizeInCellsY(); h++) { // Height of costmap
		for (int w = 0; w < costmap_->getSizeInCellsX(); w++) { // Width of costmap
			if (int(costmap_->getCost(w,h)) == costmap_2d::LETHAL_OBSTACLE) // If the position (w,h) is an obstacle
			{
				current_dist = std::sqrt(pow((pos_x - w),2) + pow((pos_y - h),2));
				//costmap_->worldToMap(obstacle_vector_.xposition,obstacle_vector_.yposition,coordx,coordy);

				//if ((std::sqrt(pow((coordx - w),2) + pow((coordy - h),2))*costmap_->getResolution()) > 2.0)
				//{
				//if (fabs(int(round(atan2(pos_y-h, pos_x-w) * 180 / PI)) - int(round(atan2(coordy, coordx) * 180 / PI))) > 30)
					// Update shortest distance
				if (current_dist < min_dist) {
					min_y = h;
					min_x = w;
					min_dist = current_dist;
				}
				//}
				/*else
				{
					std::cout << "Skipped obstacle at position: " << coordx << " " << coordy << " Obstacle position at: " << w << " " << h << endl;
					std::cout << "Distance between points is: " << std::sqrt(pow((coordx - w),2) + pow((coordy - h),2))*costmap_->getResolution() << endl;
				}*/
				
			}			
		}
	}

	std::cout << "Added obstacle at position: " << min_x << " " << min_y << endl;
	min_dist = fabs(min_dist - 1); // Makes the grid next to an obstacle to have a distance of 0 to an obstacle
	dist_x = round(pos_x - min_x);
    dist_y = round(pos_y - min_y);

    // Calculate angle
    *deg = int(round(atan2(dist_y, dist_x) * 180 / PI));

	*distance_to_obstacle = min_dist;
	shortest_distance_to_obstacle_ = min_dist;
}

/*
Function that calculates the linear and angular velocity for the robot. 
*/
void LocalPlanner::updateVelocity(tf::Vector3 force, tf::Stamped<tf::Pose> robot_pose, float *linear_velocity, float *angular_velocity, double repulsive_field_magnitude, double dipole_field_magnitude, float distance_to_path) 
{
	float u, omega, theta_d;
	//float k_omega = k2;
	float k_omega;
	//float k_u = k1;
	float k_u;
	double repulsive_field_gradient = repulsive_field_magnitude - last_repulsive_field_magnitude_; // Does the magnitude increase or decrease

	float pos_x = robot_pose.getOrigin().getX();
	float pos_y = robot_pose.getOrigin().getY();

	float theta = tf::getYaw(robot_pose.getRotation()); // Robot's heading

	float goal_x = global_goal_odom.getOrigin().getX();
	float goal_y = global_goal_odom.getOrigin().getY();
	float d = (pos_x - goal_x)*(pos_x - goal_x) + (pos_y - goal_y)*(pos_y - goal_y); d = sqrt(d); // Distance to goal

	double delta_orient = base_local_planner::getGoalOrientationAngleDifference (robot_pose, tf::getYaw(global_goal_odom.getRotation())); // Difference in rotation between robot and goal

	// Parameters that affects the attractive field
	k1 = 0.01+tanh(1/d); // The higher k1 value, the more the robot wants to stay on the path
	k2 = tanh(d)+tanh(repulsive_field_magnitude*1); // The higher k2 values, the more the robot wants to advance towards goal

	k_u = k1;
	k_omega = k2;
	
	theta_d = atan2(force.m_floats[1], force.m_floats[0]); // Desired heading

	omega = k_omega*atan2(sin(theta_d-theta),cos(theta_d-theta)); // Angular velocity to get on path

	//u = k_u*tanh(0.1*d)*tanh(1/repulsive_field_magnitude)*tanh(1/dipole_field_magnitude)*tanh(1/(distance_to_path*4));
	//u = k_u*tanh(0.1*d)*tanh(1/(distance_to_path*2))*tanh(1/(repulsive_field_magnitude*0.5))*tanh(1/(fabs(omega*2)+1));//*tanh(1/(repulsive_field_magnitude*2)); // Speed is affected by: a constant, distance to goal and the repulsive field 

	if (omega > max_angular_vel)
		omega = max_angular_vel;
	else if (omega < -max_angular_vel)
		omega = -max_angular_vel;

	if(dipole_field_magnitude < DIPOLE_FIELD_THRESHOLD) // If we do not care about the dipole field
	{
		if(repulsive_field_gradient>0) // If we're reaching towards an obstacle
			u = 1*tanh(0.5*d)*tanh(1/(repulsive_field_magnitude*1))*tanh(1/(fabs(omega*15)+1))*tanh(1/(repulsive_field_gradient*200+1e-12)); // Speed is affected by: a constant, distance to goal, the repulsive field, the angular velocity and the change of repulsive field 
		else 
			u = 1*tanh(0.5*d)*tanh(1/(repulsive_field_magnitude*1))*tanh(1/(fabs(omega*15)+1)); // Speed is affected by: a constant, distance to goal, the repulsive field and the angular velocity 
	}
	else
	{
		if(repulsive_field_gradient>0)
			u = 1*tanh(0.5*d)*tanh(1/(repulsive_field_magnitude*0.5));//tanh(1/(repulsive_field_gradient*200+1e-12)); // Speed is affected by: a constant, distance to goal and the repulsive field 
		else 
			u = 1*tanh(0.5*d)*tanh(1/(repulsive_field_magnitude*1))*tanh(1/(fabs(omega*15)+1)); // Speed is affected by: a constant, distance to goal, the repulsive field and the angular velocity
	}
	


	if ((last_linear_velocity_ + max_linear_acc_) < u)
		u = max_linear_acc_ + last_linear_velocity_;
	if (u > max_linear_vel)
		u = max_linear_vel;

	last_linear_velocity_ = u;

	if(!close_to_goal) 
	{
		*linear_velocity = u;
		*angular_velocity = omega;
	}
	else // If only the orientation is not good
	{
		float angular_vel_orientation = MAX_ANGULAR_VEL/2;
		if (delta_orient >= 0)
			*angular_velocity = angular_vel_orientation;
		else
			*angular_velocity = -angular_vel_orientation;

		*linear_velocity = 0;
	}
	std::cout << "Linear velocity: " << u << " Angular velocity: " << omega << endl;
	last_repulsive_field_magnitude_ = repulsive_field_magnitude;
	int intention_angle = -round((theta_d-theta)*180/PI);
	if (intention_angle > 180)
		intention_angle = intention_angle - 360;
	else if (intention_angle < -180)
		intention_angle = intention_angle + 360;

	std::cout << "Theta: " << theta << " Theta_d: " << theta_d << " Intention: " << intention_angle << endl; 
	intention_pub.publish(intention_angle);
}

void LocalPlanner::dipoleForce(tf::Vector3 robot_moment, tf::Stamped<tf::Pose> robot_pose, tf::Vector3 *dipole_force) 
{
	float mx, my, rx, ry, r2;
	float fx = 0;
	float fy = 0;
	float reversed_angles = (float)(M_PI + M_PI / 4);

	float theta = tf::getYaw(robot_pose.getRotation()); // Heading of the robot
	mx = last_linear_velocity_ * cos(theta); // Robot's velocity in x
	my = last_linear_velocity_ * sin(theta); // Robot's velocity in y

	//Mock data
	/*std::cout << "Test variable: " << test_variable % 4 << endl;
	if (test_variable % 4 == 0)
	{
		mx = 0;
		my = 0.4;
		rx = obstacle_vector_.xposition - 0;
		ry = obstacle_vector_.yposition - 3;
	}
	else if (test_variable % 4 == 1)
	{
		mx = 0.4;
		my = 0;
		rx = obstacle_vector_.xposition - 0;
		ry = obstacle_vector_.yposition - 3;
	}
	else if (test_variable % 4 == 2)
	{
		mx = 0;
		my = -0.4;
		rx = obstacle_vector_.xposition - 0;
		ry = obstacle_vector_.yposition - 3;
	}
	else if (test_variable % 4 == 3)
	{
		mx = -0.4;
		my = 0;
		rx = obstacle_vector_.xposition - 0;
		ry = obstacle_vector_.yposition - 3;
	}
	test_variable ++;*/
	/*mx = 0;
	my = 0.4;
	rx = obstacle_vector_.xposition - 0;
	ry = obstacle_vector_.yposition - 2;*/
	// -- Mock data

	// Equation 7 - Dynamic Dipole Field
	// d^ = d / |d|
	//std::cout << "Robot pos: " << robot_pose.getOrigin().getX() << " " << robot_pose.getOrigin().getY();
	//std::cout << " Obst pos: " << obstacle_vector_.xposition << " " << obstacle_vector_.yposition << endl;
	//std::cout << "Robot vel: " << mx << " " << my;
	//std::cout << " Obst vel:" << obstacle_vector_.xvelocity << " " << obstacle_vector_.yvelocity << endl;

	rx = obstacle_vector_.xposition - robot_pose.getOrigin().getX();
	ry = obstacle_vector_.yposition - robot_pose.getOrigin().getY();
	r2 = rx*rx + ry*ry + (1e-12);
	rx /= std::sqrt(r2);
	ry /= std::sqrt(r2);

	//(Mj * d^)Mk
	fx += (obstacle_vector_.xvelocity*rx + obstacle_vector_.yvelocity*ry) * mx;
	fy += (obstacle_vector_.xvelocity*rx + obstacle_vector_.yvelocity*ry) * my;

	//(Mk * d^)Mj
	fx += (mx*rx + my*ry) * obstacle_vector_.xvelocity;
	fy += (mx*rx + my*ry) * obstacle_vector_.yvelocity;

	//(Mj * Mk)d^
	fx += (mx*obstacle_vector_.xvelocity + my*obstacle_vector_.yvelocity) * rx;
	fy += (mx*obstacle_vector_.xvelocity + my*obstacle_vector_.yvelocity) * ry;

	// 5(Mj * d^)(Mk * d^)d^
	fx -= 5*(mx*rx + my*ry) * (obstacle_vector_.xvelocity*rx + obstacle_vector_.yvelocity*ry) * rx;
	fy -= 5*(mx*rx + my*ry) * (obstacle_vector_.xvelocity*rx + obstacle_vector_.yvelocity*ry) * ry;

	//3p/d^4g (...) Equation 8 - Scale the dipole field
	// default: 1000 * fx / pow(r2*r2, float(GAMA));
	fx = 1000 * fx / pow(r2*r2, float(GAMA));
	fy = 1000 * fy / pow(r2*r2, float(GAMA));

	float sign = (fx*rx + fy*ry);
	//if (sign > 0)
	//	dipole_force->setValue(-fx,-fy,0);
	dipole_force->setValue(fx,fy,0);
	//std::cout << "Original dipole force: " << dipole_force->m_floats[0] << " " << dipole_force->m_floats[1] << " " << dipole_force->m_floats[2] << endl;
	if (sign < 0)
	{
		dipole_force->setValue(fx,fy,0);
		//ROS_INFO("Sign negative, negative");
		if (fx*rx > 0)
		{
			dipole_force->setValue(fx,fy,0); //-fx,fy
			//ROS_INFO("Changed nothing");
		}
		else if (fy*ry > 0)
		{
			dipole_force->setValue(fx,fy,0);
			//ROS_INFO("Changed fy");
		}
		/*if (fx*rx < 0)
		{
			dipole_force->setValue(fx,fy,0);
			ROS_INFO("Sign negative, negative");
		}
		else
		{
			dipole_force->setValue(-fx,-fy,0);
			ROS_INFO("Sign negative, positive");
		}
		*/
	}
	else
	{
		//ROS_INFO("Sign positive");
		//std::cout << "If negative: " << fx << " " << fy << endl;
		dipole_force->setValue(-fx,-fy,0);
		if (fx*rx < 0)
		{
			dipole_force->setValue(-fx,-fy,0);
			//ROS_INFO("Changed fx");
		}
		else if (fy*ry < 0)
		{
			dipole_force->setValue(-fx,-fy,0);
			//ROS_INFO("Changed fy");
		}
		//std::cout << "Alternative dipole force: " << fx*cos(reversed_angles)-fx*sin(reversed_angles) << " " << fy*sin(reversed_angles)+fy*cos(reversed_angles) << endl;
		//dipole_force->setValue(fx*cos(reversed_angles)-fx*sin(reversed_angles),fx*sin(reversed_angles)+fx*cos(reversed_angles),0);
		//dipole_force->setValue(fx*cos(reversed_angles)-fx*sin(reversed_angles),fy*sin(reversed_angles)+fy*cos(reversed_angles),0);
	}
		
	

	if(isnan(dipole_force->m_floats[0]))
		dipole_force->setValue(0,0,0);	

	//std::cout << "Dipole force: " << dipole_force->m_floats[0] << " " << dipole_force->m_floats[1] << " " << dipole_force->m_floats[2] << endl;
}

/*
Callback function that listens to the object tracking node
*/
void LocalPlanner::dynamicObstacleCallback(const vector_creation::vector msg)
{
	if(msg.xvelocity != 0 && msg.yvelocity != 0) // If the message is not empty
		obstacle_vector_ = msg; // Copy to a private variable
}

/*
Function that calculates everything that the navigation part needs
*/
bool LocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){

	
	double trajCost,minCost,x,y,th;
	float angular_velocity, linear_velocity;

	if(!initialized_)
	{
		ROS_ERROR("MDH Local_Planner has not been initialized, please call initialize() before using this planner");
		return false;
	}

	//ros::Time now = ros::Time::now();
 	//unsigned int  now.sec

	//Get where we are
	tf::Stamped<tf::Pose> robot_pose;
	costmap_ros_->getRobotPose(robot_pose);	
	int coord_x, coord_y;

	int plan_index = 0; // Will eventually hold the index belonging to the global path which is closest to the robot
    float distance_to_path; // Will eventually hold the distance to the closest point on path
    tf::Vector3 f_attractive_vector, f_repulsive_vector, final_vector, dipole_force, vector_to_compute, obstacle_velocity_vector;
	global_plan_temp = global_plan_;
	generate_new_path = 1;
    geometry_msgs::Point fa_point_path = closestPointOnPath(robot_pose, &plan_index, &distance_to_path); // Where are we

	//ros::spinOnce(); // Receive information from callback

	repulsiveForce(robot_pose, &f_repulsive_vector);

    if (distance_to_path <= 1 || shortest_distance_to_obstacle_ <= 6.1) // Inside bounds and not too close to an obstacle
    {
		// Attractive vector equations
		tf::Vector3 unit_vector_ni(unitVectorOfPath(plan_index));
		fAttractiveVector(fa_point_path, robot_pose, unit_vector_ni, &f_attractive_vector);
		f_attractive_vector.operator*=(1 - exp(-k1*distance_to_path)); 
		f_attractive_vector.operator+=(unit_vector_ni.operator*=(k2*exp(-k1*distance_to_path)));

		dipoleForce(final_vector, robot_pose, &dipole_force);

		if (f_repulsive_vector.length() > REPULSIVE_FIELD_THRESHOLD && dipole_force.length() > DIPOLE_FIELD_THRESHOLD)
		{
			f_repulsive_vector.normalize();
			f_repulsive_vector.operator*=(2);
			dipole_force.normalize();
			f_attractive_vector.operator/=(10);
		}			
		else if (dipole_force.length() <= DIPOLE_FIELD_THRESHOLD)
			dipole_force.setValue(0,0,0);
		else
		{
			dipole_force.normalize();
			dipole_force.operator*=(2); 
		}

		final_vector = f_repulsive_vector;
		final_vector.operator+=(dipole_force);
		final_vector.operator+=(f_attractive_vector);

		std::cout << "Dipole force: " << dipole_force.m_floats[0] << " " << dipole_force.m_floats[1] << " " << dipole_force.m_floats[2] << endl;
		std::cout << "Repulsive force: " << f_repulsive_vector.m_floats[0] << " " << f_repulsive_vector.m_floats[1] << " " << f_repulsive_vector.m_floats[2] << endl;
		std::cout << "Final vector: " << final_vector.m_floats[0] << " " << final_vector.m_floats[1] << " " << final_vector.m_floats[2] << endl;

		/*double mapx, mapy;
		costmap_->mapToWorld(costmap_->getSizeInCellsX()/2,costmap_->getSizeInCellsY()/2,mapx,mapy);
		std::cout << "Middle of costmap: " << mapx << " " << mapy << endl;

		unsigned int coordx,coordy;
		costmap_->worldToMap(global_goal_odom.getOrigin().getX(),global_goal_odom.getOrigin().getY(),coordx,coordy);
		std::cout << "Cost of goal: " << (int)costmap_->getCost(coordx,coordy) << endl;*/

		updateVelocity(final_vector, robot_pose, &linear_velocity, &angular_velocity, f_repulsive_vector.length(), dipole_force.length(), distance_to_path);
		cmd_vel.angular.z = angular_velocity;
		cmd_vel.linear.x = linear_velocity;
		//intention_pub.publish();
		
		minCost = 255;//std::numeric_limits<double>::max();
		global_plan_temp.clear();
	}
	else
	{
		geometry_msgs::PoseStamped check_if_done_ = global_plan_.back();
		new_map_pub.publish(check_if_done_);
		generate_new_path = 1; // Re-calculate global plan. Out of bounds
	}
	
	return true;
}

/*
Checks of the goal is reached
*/
bool LocalPlanner::isGoalReached(){

	if (!goal_reached_)
	{
		tf::Stamped<tf::Pose> robot_pose;
		costmap_ros_->getRobotPose(robot_pose); //frame_id_=odom 

		//Both, global goal and robot pose are in the same coordinate system, odom.
		double dx = global_goal_odom.getOrigin().getX() - robot_pose.getOrigin().getX();
		double dy = global_goal_odom.getOrigin().getY() - robot_pose.getOrigin().getY();

		double delta_orient = base_local_planner::getGoalOrientationAngleDifference (robot_pose, tf::getYaw(global_goal_odom.getRotation()));

		unsigned int coordx = 0,coordy = 0;

		costmap_->worldToMap(global_goal_odom.getOrigin().getX(),global_goal_odom.getOrigin().getY(),coordx,coordy);

		//std::cout << "Goal coordinate costmap: " << coordx << " " << coordy << endl;
		//std::cout << "Final orientation: " << tf::getYaw(global_goal_odom.getRotation()) << endl;
		//std::cout << "Cost: " << ((int)costmap_->getCost(coordx,coordy)) << " At position: " << coordx << " " << coordy << endl;

		if (fabs(std::sqrt(dx*dx+dy*dy)) < 1.0)
		{
			if (((int)costmap_->getCost(coordx,coordy)) > 0)
			{
				ROS_INFO("Could not fully reach the goal. The goal is too close to an obstacle at %f %f", global_goal_odom.getOrigin().getX(), global_goal_odom.getOrigin().getY());
				goal_reached_ = true;
				close_to_goal = false;
				geometry_msgs::PoseStamped check_if_done_;
				check_if_done_.header.frame_id = global_frame_;
				check_if_done_.header.stamp = ros::Time::now();
				check_if_done_.pose.position.x = -2500000.0;
				check_if_done_.pose.position.y = -2500000.0;
				check_if_done_.pose.position.z = 0.0;
				check_if_done_.pose.orientation.x = 0.0;
				check_if_done_.pose.orientation.y = 0.0;
				check_if_done_.pose.orientation.z = 0.0;
				check_if_done_.pose.orientation.w = 1.0;
				new_map_pub.publish(check_if_done_);
				generate_new_path = 1;
				return true;
			}		
			else if (fabs(std::sqrt(dx*dx+dy*dy)) < 0.2 && fabs(delta_orient) < (30 * PI / 180))
			{
				goal_reached_ = true;
				ROS_INFO("Goal reached close to %f %f", global_goal_odom.getOrigin().getX(), global_goal_odom.getOrigin().getY());
				close_to_goal = false;
				geometry_msgs::PoseStamped check_if_done_;
				check_if_done_.header.frame_id = global_frame_;
				check_if_done_.header.stamp = ros::Time::now();
				check_if_done_.pose.position.x = -2500000.0;
				check_if_done_.pose.position.y = -2500000.0;
				check_if_done_.pose.position.z = 0.0;
				check_if_done_.pose.orientation.x = 0.0;
				check_if_done_.pose.orientation.y = 0.0;
				check_if_done_.pose.orientation.z = 0.0;
				check_if_done_.pose.orientation.w = 1.0;
				new_map_pub.publish(check_if_done_);
				generate_new_path = 1;
				return true;

			}
			else if (fabs(std::sqrt(dx*dx+dy*dy)) < 0.1) // If we only need to adjust the orientation
			{
				close_to_goal = true;
			}
			else
				close_to_goal = false;
		}
	}
   	return goal_reached_;
}

/*
Sets the global plan received from the global planner
*/
bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
   	if(!initialized_)
	{
		ROS_ERROR("Please call initialize() before using this planner");
		return false;
	}
	if (plan.size() > 1)
	{
		std::cout << "Plan not empty " << " Size of plan: " << plan.size() << endl;
		if(generate_new_path == 1)
		{
			global_plan_.clear();
			global_plan_ = plan;

			//Get where we are
			tf::Stamped<tf::Pose> robot_pose;
			costmap_ros_->getRobotPose(robot_pose);	


			tf::poseStampedMsgToTF(global_plan_.back(), global_goal);

			try{
				tf_.transformPose(global_frame_, global_goal, global_goal_odom); ////transform the global goal from map to odom 
				//tf_.waitForTransform(global_frame_, global_goal, global_goal_odom); ////transform the global goal from map to odom 
			}
			catch (tf::TransformException ex){
				ROS_ERROR("IsGoalReached %s",ex.what());
				return false;
			}
			close_to_goal = false;
			goal_reached_ = false;
			generate_new_path = 0;
		}
	}
	


	return true;
}

/*
Initialize Local planner
*/
 void LocalPlanner::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros){
  
   	// check if the plugin is already initialized
      if(!initialized_)
      {
        // copy adress of costmap and Transform Listener (handed over from move_base)
        costmap_ros_ = costmap_ros;
	costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.


    	global_frame_ = costmap_ros_->getGlobalFrameID();
	robot_base_frame_ = costmap_ros_->getBaseFrameID();

        // create Node Handle with name of plugin (as used in move_base for loading)
        ros::NodeHandle pn("~/" + name);
		ros::NodeHandle path_handle;
		ros::NodeHandle intention_node;

        // advertise topics (adapted global plan and predicted local trajectory)
        l_plan_pub_ = pn.advertise<nav_msgs::Path>("local_plan", 1);
	marker_pub = pn.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	shape = visualization_msgs::Marker::CUBE;
	new_map_pub	= path_handle.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1);
	intention_pub = intention_node.advertise<std_msgs::Int32>("TX2_localplanner_intention",1);



	// Added by Peter
	std::cout << "Local costmap: " << endl;
	std::cout << "Resolution: " << costmap_->getResolution() << endl;
	std::cout << "Size in cells x : " << costmap_->getSizeInCellsX() << endl;
	std::cout << "Size in cells y : " << costmap_->getSizeInCellsY() << endl;
	std::cout << "Origin costmap: " << costmap_->getOriginX() << " " << costmap_->getOriginY() << endl;
	double mapx, mapy;
	//costmap_->mapToWorld(costmap_->getOriginX(),costmap_->getOriginY(),mapx,mapy);
	costmap_->mapToWorld(0,0,mapx,mapy);
	std::cout << "Origin costmap world coordinates: " << mapx << " " << mapy << endl;
	costmap_->mapToWorld(costmap_->getSizeInCellsX()/2,costmap_->getSizeInCellsY()/2,mapx,mapy);
	std::cout << "World coordinate in middle of costmap: " << mapx << " " << mapy << endl;
	test_variable = 0;
	generate_new_path = 1;
	last_linear_velocity_ = 0;
	k1 = 0.01;
	k2 = 1;
	ros::NodeHandle moving_obstacle_node;
	moving_obstacle_subscriber_ = moving_obstacle_node.subscribe<vector_creation::vector>("/vector_generation/vector", 1, &LocalPlanner::dynamicObstacleCallback, this);
	close_to_goal = false;
	final_vectors_vector.clear();
	last_repulsive_field_magnitude_ = 0.0;
	shortest_distance_to_obstacle_ = 100.0;
	// -- Peter


        // set initialized flag
        initialized_ = true;

	

        // this is only here to make this process visible in the rxlogger right from the start
        ROS_DEBUG("Local planner plugin initialized.");
      }
      else
      {
        ROS_WARN("This planner has already been initialized, doing nothing.");
      }
  }

 };
