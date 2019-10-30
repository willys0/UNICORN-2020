#include <pluginlib/class_list_macros.h>
#include "local_planner.h"

//#define INT_MAX 2000
#define PI 3.14159265

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
    int gain = 106; // 2000 // Default 10000
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
	//float* p_distance_to_obstacle = &distance_to_obstacle;
	findClosestObjectEuclidean(deg, &distance_to_obstacle);

	d = distance_to_obstacle;
    d2 = d / scale + 1;
    d = (1 / d2 - d0);
    d = d*d;

	//std::cout << "d2: " << d2 << " dmax: " << dmax << " distance to obstacle: " << distance_to_obstacle << endl; 

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

	int pos_x = costmap_->getSizeInCellsX() / 2; // Center of the costmap (Robot's position)
	int pos_y = costmap_->getSizeInCellsY() / 2;
	
	for (int h = 0; h < costmap_->getSizeInCellsY(); h++) { // Height of costmap
		for (int w = 0; w < costmap_->getSizeInCellsX(); w++) { // Width of costmap
			if (int(costmap_->getCost(w,h)) == costmap_2d::LETHAL_OBSTACLE) // If the position (w,h) is an obstacle
			{
				current_dist = std::sqrt(pow((pos_x - w),2) + pow((pos_y - h),2));

				// Update shortest distance
				if (current_dist < min_dist) {
					min_y = h;
					min_x = w;
					min_dist = current_dist;
				}
			}			
		}
	}

	min_dist = fabs(min_dist - 1); // Makes the grid next to an obstacle to have a distance of 0 to an obstacle
	dist_x = round(pos_x - min_x);
    dist_y = round(pos_y - min_y);

    // Calculate angle
    *deg = int(round(atan2(dist_y, dist_x) * 180 / PI));

	*distance_to_obstacle = min_dist;
}

/*
Function that calculates the linear and angular velocity for the robot. 
*/
void LocalPlanner::updateVelocity(tf::Vector3 force, tf::Stamped<tf::Pose> robot_pose, float *linear_velocity, float *angular_velocity, double repulsive_field_magnitude) 
{
	float u, omega, theta_d;
	//float k_omega = k2;
	float k_omega;
	//float k_u = k1;
	float k_u;

	float pos_x = robot_pose.getOrigin().getX();
	float pos_y = robot_pose.getOrigin().getY();

	float theta = tf::getYaw(robot_pose.getRotation());// + PI;

	float goal_x = global_goal_odom.getOrigin().getX();
	float goal_y = global_goal_odom.getOrigin().getY();
	float d = (pos_x - goal_x)*(pos_x - goal_x) + (pos_y - goal_y)*(pos_y - goal_y); // Distance to goal
	d = sqrt(d);

	k1 = 1;//tanh(0.1*d);
	k2 = 2-tanh(d/2);

	/*if(d < 1)
	{
		k1 = 0.01;
		k2 = 2;
	}
	else
	{
		k1 = 1;
		k2 = 1;
	}*/
	k_u = k1;
	k_omega = k2;
	
		

	u = k_u*tanh(0.1*d);//*tanh(1/(repulsive_field_magnitude*2)); // Speed is affected by: a constant, distance to goal and the repulsive field 

	theta_d = atan2(force.m_floats[1], force.m_floats[0]);
	//omega = -k_omega*(theta - atan2(force.m_floats[1], force.m_floats[0])); // Angular velocity is affected by the current orientation and the wanted orientation
	//omega = -k_omega*(theta - atan2(force.m_floats[1], force.m_floats[0])); // Angular velocity is affected by the current orientation and the wanted orientation
	//omega = -k_omega*(theta - theta_d); // Angular velocity is affected by the current orientation and the wanted orientation

	omega = k_omega*atan2(sin(theta_d-theta),cos(theta_d-theta));
	//std::cout << "Theta: " << theta << " Theta_d: " << theta_d << " Omega: " << omega << " Test omega: " << test_omega << endl;

	if ((last_linear_velocity_ + max_linear_acc_) < u)
		u = max_linear_acc_ + last_linear_velocity_;
	if (u > 0.4)
		u = 0.4;
	if (omega > 0.4)
		omega = 0.4;
	last_linear_velocity_ = u;
	std::cout << "Linear velocity: " << u << " Angular velocity: " << omega << endl;
	//std::cout << "Angular velocity: " << omega << endl;

	*linear_velocity = u;
	*angular_velocity = omega;
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

	//Get where we are
	tf::Stamped<tf::Pose> robot_pose;
	costmap_ros_->getRobotPose(robot_pose);	
	int coord_x, coord_y;

	int plan_index = 0; // Will eventually hold the index belonging to the global path which is closest to the robot
    float distance_to_path; // Will eventually hold the distance to the closest point on path
    tf::Vector3 f_attractive_vector, f_repulsive_vector, final_vector;
	global_plan_temp = global_plan_;
	generate_new_path = 1;
    geometry_msgs::Point fa_point_path = closestPointOnPath(robot_pose, &plan_index, &distance_to_path);
    if (distance_to_path <= 5) // Inside bounds
    {
		tf::Vector3 unit_vector_ni(unitVectorOfPath(plan_index));

		fAttractiveVector(fa_point_path, robot_pose, unit_vector_ni, &f_attractive_vector);
		f_attractive_vector.operator*=(1 - exp(-k1*distance_to_path)); 
		f_attractive_vector.operator+=(unit_vector_ni.operator*=(k2*exp(-k1*distance_to_path)));

		repulsiveForce(robot_pose, &f_repulsive_vector);

		final_vector = f_attractive_vector;
		final_vector.operator+=(f_repulsive_vector);

		
		//std::cout << "Attractive vector " << f_attractive_vector.m_floats[0] << " " << f_attractive_vector.m_floats[1] << " " << f_attractive_vector.m_floats[2] << endl;
		std::cout << " Repulsive vector: " << f_repulsive_vector.m_floats[0] << " " << f_repulsive_vector.m_floats[1] << " " << f_repulsive_vector.m_floats[2];
		std::cout << " Final vector: " << final_vector.m_floats[0] << " " << f_attractive_vector.m_floats[1] << " " << f_attractive_vector.m_floats[2] << endl;
		

		updateVelocity(final_vector, robot_pose, &linear_velocity, &angular_velocity, f_repulsive_vector.length());
		cmd_vel.angular.z = angular_velocity;
		cmd_vel.linear.x = linear_velocity;
		
		minCost = 255;//std::numeric_limits<double>::max();
		global_plan_temp.clear();
	}
	else
		generate_new_path = 1; // Re-calculate global plan. Out of bounds
	
	return true;
}

/*
Checks of the goal is reached
*/
bool LocalPlanner::isGoalReached(){
	//Get the actual Costmap
	tf::Stamped<tf::Pose> robot_pose;
	costmap_ros_->getRobotPose(robot_pose); //frame_id_=odom 

	//Both, globla goal and robot pose are in the same coordinate system, odom.
	double dx = global_goal_odom.getOrigin().getX() - robot_pose.getOrigin().getX();
	double dy = global_goal_odom.getOrigin().getY() - robot_pose.getOrigin().getY();

	double delta_orient = base_local_planner::getGoalOrientationAngleDifference (robot_pose, tf::getYaw(global_goal_odom.getRotation()));

	//std::cout << "Robot pos: " << robot_pose.getOrigin().getX() << " " << robot_pose.getOrigin().getY() << endl;
	//std::cout << "Goal pos : " << global_goal_odom.getOrigin().getX() << " " << global_goal_odom.getOrigin().getY() << endl;
	if(fabs(std::sqrt(dx*dx+dy*dy)) < 0.2 && fabs(delta_orient) < (30 * PI / 180))
	{
		goal_reached_ = true;
		ROS_INFO("Goal reached close to %f %f", global_goal_odom.getOrigin().getX(), global_goal_odom.getOrigin().getY());
		generate_new_path = 1;
		return true;

	}
	
   	return false;
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

		goal_reached_ = false;
	}
	generate_new_path = 0;


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

        // advertise topics (adapted global plan and predicted local trajectory)
        l_plan_pub_ = pn.advertise<nav_msgs::Path>("local_plan", 1);
	marker_pub = pn.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	shape = visualization_msgs::Marker::CUBE;


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
