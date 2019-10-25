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
	
	std::vector<geometry_msgs::PoseStamped> transformed_plan;
	if(base_local_planner::transformGlobalPlan(tf_,global_plan_temp,robot_pose,*costmap_,global_frame_,transformed_plan)) //Transforme the plan respect to the robot base
{
	int i = 0;
	for(std::vector<geometry_msgs::PoseStamped>::iterator it = transformed_plan.begin(); it != transformed_plan.end(); ++it) 
	{
    	dx =  it->pose.position.x-robot_pose.getOrigin().getX();
		dy =  it->pose.position.y-robot_pose.getOrigin().getY();
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
		i++;
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
    float x_value, y_value;
    pair<float,float> vector_value;
	//std::cout << "Global_plan_temp size: " << global_plan_temp.size() << " Plan index: " << plan_index << endl;
    first_point = global_plan_temp[plan_index];
	//std::cout << "unit_vector1" << endl;
    if(global_plan_temp.size() == (plan_index + 1))
	{
		//global_plan_temp[i].pose.position.x
		/*geometry_msgs::PoseStamped temp;
		temp.pose.position.x = global_plan_temp[plan_index].pose.position.x - global_plan_temp[plan_index-1].pose.position.x;
		temp.pose.position.y = global_plan_temp[plan_index].pose.position.y - global_plan_temp[plan_index-1].pose.position.y;
		first_point.pose.position.x = global_plan_temp[plan_index].pose.position.x + temp.pose.position.x;
		first_point.pose.position.y = global_plan_temp[plan_index].pose.position.y + temp.pose.position.y;
		second_point = global_plan_temp[plan_index];
		*/
		first_point = global_plan_temp[plan_index-1];
		second_point = global_plan_temp[plan_index];
        //second_point = global_plan_temp[plan_index];
		//second_point = global_plan_temp[plan_index-1];
		//std::cout << "unit_vector2" << endl;
	}
    else
	{
        second_point = global_plan_temp[plan_index+1];
	}
    x_value = second_point.pose.position.x - first_point.pose.position.x;
    y_value = second_point.pose.position.y - first_point.pose.position.y;
    tf::Vector3 vector_heading(x_value, y_value, 0);
    vector_heading.normalize();
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

    // Mock values
    /*fa_point_path.x = 1;
    fa_point_path.y = 1;
    fa_point_path.z = 0;
    unit_vector_ni.m_floats[0] = 1;
    unit_vector_ni.m_floats[1] = 1;
    unit_vector_ni.m_floats[2] = 0;*/
    // -- Mock values

    tf::Vector3 robot_position(robot_pose.getOrigin().getX(), robot_pose.getOrigin().getY(), 0);
    tf::Vector3 ai(fa_point_path.x ,fa_point_path.y, 0);
    tf::Vector3 ai_p = ai.operator-=(robot_position);
    tfScalar attractive_scalar = ai_p.dot(unit_vector_ni);
    *attractive_vector = ai_p.operator-=(unit_vector_ni.operator*=(attractive_scalar));
    attractive_vector->normalize();
    if(isnan(attractive_vector->m_floats[0]))
    {
        attractive_vector->setValue(0.0,0.0,0.0);
    }
}

void LocalPlanner::repulsiveForce(tf::Stamped<tf::Pose> robot_pose, tf::Vector3 *repulsive_vector)
{
    int xt, yt, cost, force_x, force_y, deg;
    int scale = 100;
    int gain = 10000;
    float dmax = 1.25;
	float repulsive_force, x_force, y_force, z_force;

	makeRepulsiveField(scale, gain, dmax, robot_pose.getOrigin().getX(), robot_pose.getOrigin().getY(), &repulsive_force, &deg);
    
	if (repulsive_force > 0)
	{
		y_force = repulsive_force*sin(deg*PI/180);
		x_force = repulsive_force*cos(deg*PI/180);
		z_force = 0;
		repulsive_vector->setValue(x_force,y_force,z_force);
	}
	else
	{
		repulsive_vector->setValue(0.0,0.0,0.0);
	}
}

void LocalPlanner::makeRepulsiveField(int scale, int gain, float dmax, float pos_x, float pos_y, float *repulsive_force, int *deg)
{
    float d, d0, d2;
    d0 = 1.0/dmax;
	float distance_to_obstacle;
	findClosestObjectEuclidean(deg, &distance_to_obstacle);

	d = distance_to_obstacle;
    d2 = d / scale + 1;
    d = (1 / d2 - d0);
    d = d*d;

	if (d2 <= dmax)
		*repulsive_force = gain*d;
	else
		*repulsive_force = 0;
}

void LocalPlanner::findClosestObjectEuclidean(int *deg, float *distance_to_obstacle)
{
    float min_dist = 2000.0;
    float current_dist = 0;

    int current_point = 0;

    int min_x = 0;
    int min_y = 0;

    int dist_x = 0;
    int dist_y = 0;	

	int pos_x = costmap_->getSizeInCellsX() / 2;
	int pos_y = costmap_->getSizeInCellsY() / 2;
	
	for (int h = 0; h < costmap_->getSizeInCellsY(); h++) {
		for (int w = 0; w < costmap_->getSizeInCellsX(); w++) {
			if (int(costmap_->getCost(w,h)) == costmap_2d::LETHAL_OBSTACLE)
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
	

	// Mock Values

	//min_dist = 35.0;
	//*deg = 90;

	// -- Mock Values

	min_dist = fabs(min_dist - 1);
	dist_x = round(pos_x - min_x);
    dist_y = round(pos_y - min_y);

    // Calculate angle
    *deg = int(round(atan2(dist_y, dist_x) * 180 / PI));

	*distance_to_obstacle = min_dist;
}

void LocalPlanner::updateVelocity(tf::Vector3 force, tf::Stamped<tf::Pose> robot_pose, float *linear_velocity, float *angular_velocity) 
{
	float u, omega;
	float k_omega = k2;
	float k_u = k1;

	float pos_x = robot_pose.getOrigin().getX();
	float pos_y = robot_pose.getOrigin().getY();

	float theta = tf::getYaw(robot_pose.getRotation());

	float goal_x = global_goal_odom.getOrigin().getX();
	float goal_y = global_goal_odom.getOrigin().getY();
	float d = (pos_x - goal_x)*(pos_x - goal_x) + (pos_y - goal_y)*(pos_y - goal_y);
	d = sqrt(d);// / 20;
	u = 20*k_u*tanh(d);

	omega = -k_omega*(theta - atan2(force.m_floats[1], force.m_floats[0]));

	*linear_velocity = u;
	*angular_velocity = omega;
}

//Return true if a admisible solution is found
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

	int plan_index = 0;
    float distance_to_path;
    tf::Vector3 f_attractive_vector, f_repulsive_vector, final_vector;
	global_plan_temp = global_plan_;

    geometry_msgs::Point fa_point_path = closestPointOnPath(robot_pose, &plan_index, &distance_to_path);
    if (distance_to_path > 100)
    {
        // Re-calculate Thetastar
    }

    tf::Vector3 unit_vector_ni(unitVectorOfPath(plan_index));

    fAttractiveVector(fa_point_path, robot_pose, unit_vector_ni, &f_attractive_vector);
    f_attractive_vector.operator*=(1 - exp(-k1*distance_to_path)); 
    f_attractive_vector.operator+=(unit_vector_ni.operator*=(k2*exp(-k1*distance_to_path)));

	repulsiveForce(robot_pose, &f_repulsive_vector);

	final_vector = f_attractive_vector.operator+=(f_repulsive_vector);

	//std::cout << "Repulsive vector " << f_repulsive_vector.m_floats[0] << " " << f_repulsive_vector.m_floats[1] << " " << f_repulsive_vector.m_floats[2] << endl;
	std::cout << "Final vector " << final_vector.m_floats[0] << " " << f_attractive_vector.m_floats[1] << " " << f_attractive_vector.m_floats[2] << endl;


	updateVelocity(final_vector, robot_pose, &linear_velocity, &angular_velocity);
	cmd_vel.angular.z = angular_velocity;
	cmd_vel.linear.x = linear_velocity;
	
	minCost = 255;//std::numeric_limits<double>::max();
	global_plan_temp.clear();

	return true;
}

bool LocalPlanner::isGoalReached(){
	//Get the actual Costmap
	tf::Stamped<tf::Pose> robot_pose;
	costmap_ros_->getRobotPose(robot_pose); //frame_id_=odom 
		

	//Both, globla goal and robot pose are in the same coordinate system, odom.
	double dx = global_goal_odom.getOrigin().getX() - robot_pose.getOrigin().getX();
	double dy = global_goal_odom.getOrigin().getY() - robot_pose.getOrigin().getY();

	double delta_orient = base_local_planner::getGoalOrientationAngleDifference (robot_pose, tf::getYaw(global_goal_odom.getRotation()));

	if(fabs(std::sqrt(dx*dx+dy*dy)) < 0.2) //&& fabs(delta_orient) < 0.2)
	{
		goal_reached_ = true;
		ROS_INFO("Goal reached!");
		return true;

	}
	
   	return false;
}

bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
   	if(!initialized_)
	{
		ROS_ERROR("Please call initialize() before using this planner");
		return false;
	}

	global_plan_.clear();
	global_plan_ = plan;

	//Get where we are
	tf::Stamped<tf::Pose> robot_pose;
	costmap_ros_->getRobotPose(robot_pose);	


  	tf::poseStampedMsgToTF(global_plan_.back(), global_goal);

	try{
		tf_.transformPose(global_frame_, global_goal, global_goal_odom); ////transform the global goal from map to odom 
	}
	catch (tf::TransformException ex){
		ROS_ERROR("IsGoalReached %s",ex.what());
		return false;
	}

	goal_reached_ = false;


	return true;
}

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

	std::cout << "Resolution: " << costmap_->getResolution() << endl;
	std::cout << "Size in cells x : " << costmap_->getSizeInCellsX() << endl;
	std::cout << "Size in cells y : " << costmap_->getSizeInCellsY() << endl;
	costmap_2d::Costmap2D costmap_peter(*costmap_);
	std::cout << "Resolution: " << costmap_peter.getResolution() << endl;
	std::cout << "Size in cells x : " << costmap_peter.getSizeInCellsX() << endl;
	std::cout << "Size in cells y : " << costmap_peter.getSizeInCellsY() << endl;
	std::string map_file = "/home/ros2/catkin_ws_peter/test.pgm";
	//geometry_msgs::PointStamped peters_point;
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

//Given a position (x=pos[0], y=pos[1], theta=pos[3]), an angular (vel[1]) and a linear (vel[0]) velocities, and period of time, this function calculate where the robot will be after dt seconds in the future (differential robot).
Eigen::Vector3f LocalPlanner::computeNewPositions(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel) {
	Eigen::Vector3f new_pos = Eigen::Vector3f::Zero();
	new_pos[0] = pos[0] + (vel[0] * cos(pos[2]) + vel[1] * cos(M_PI_2 + pos[2])) * DT;
	new_pos[1] = pos[1] + (vel[0] * sin(pos[2]) + vel[1] * sin(M_PI_2 + pos[2])) * DT;
	new_pos[2] = pos[2] + vel[2] * DT;
	return new_pos;
}

// Given a position of the robot (robot_pose), linear and angular velocities, and the simulation time, this function discretize the  trajectory that the robot will follow in a vector of points (base_local_planner::Trajectory).
base_local_planner::Trajectory LocalPlanner::generateTrajectory(tf::Stamped<tf::Pose> robot_pose, double linear, double angular, double t)
{
	//Sampling rate 1/dt
	int numPoints = 2*t/DT;
	
	base_local_planner::Trajectory traj(linear,0,angular, DT,numPoints); 
	traj.resetPoints();

	Eigen::Vector3f pos,vel;
	pos[0] = robot_pose.getOrigin().getX();
	pos[1] = robot_pose.getOrigin().getY();
	pos[2] = tf::getYaw(robot_pose.getRotation());

	vel[0] = linear;
	vel[1] = 0;
	vel[2] = angular;

	traj.addPoint(pos[0], pos[1], pos[2]);
	//ROS_INFO("Initial pos (%2.2f, %2.2f, %2.2f) %2.2f,%2.2f",pos[0],pos[1],pos[2],linear,angular);
	
	for (double cont = 0; cont < t; cont+=DT) //t seconds simulation
	{
		pos = computeNewPositions(pos,vel);
		traj.addPoint(pos[0], pos[1], pos[2]);
		//ROS_INFO("pos (%2.2f, %2.2f, %2.2f)",pos[0],pos[1],pos[2]);
	}
	
	return traj;

}

//Prune global plan and search the local goal
tf::Stamped<tf::Pose> LocalPlanner::findLocalGoal(tf::Stamped<tf::Pose> robot_pose)
{
	std::vector<geometry_msgs::PoseStamped> transformed_plan;
	tf::Stamped<tf::Pose> local_goal;
	float dx,dy;

	base_local_planner::transformGlobalPlan(tf_,global_plan_,robot_pose,*costmap_,global_frame_,transformed_plan); //Transforme the plan respect to the robot base


	//tf::poseStampedMsgToTF(transformed_plan.back(), local_goal);

	bool beforeRobot = true;
	bool findGoal = false;
	for(std::vector<geometry_msgs::PoseStamped>::iterator it = transformed_plan.begin(); it != transformed_plan.end(); ++it) {
    		dx =  it->pose.position.x-robot_pose.getOrigin().getX();
		dy =  it->pose.position.y-robot_pose.getOrigin().getY();
		if(fabs(std::sqrt(dx*dx+dy*dy)) < 1){
			beforeRobot=false;
		}
		else if ((fabs(std::sqrt(dx*dx+dy*dy)) > 1) && !beforeRobot)
		{
			tf::poseStampedMsgToTF(*it, local_goal);
			findGoal=true;
			break;
		}
	}
	
	if(!findGoal) tf::poseStampedMsgToTF(transformed_plan.back(), local_goal);
	drawLocalGoal(local_goal);
	
	return local_goal;
}

void LocalPlanner::drawLocalGoal(tf::Stamped<tf::Pose> local_goal){
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = global_frame_;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = shape;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = local_goal.getOrigin().getX();
	marker.pose.position.y = local_goal.getOrigin().getY();
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration();
	//------
	marker_pub.publish(marker);
}

 };
