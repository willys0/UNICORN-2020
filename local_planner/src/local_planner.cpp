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

geometry_msgs::Point LocalPlanner::closestPointOnPathMatrix(tf::Stamped<tf::Pose> robot_pose, int *plan_index, float *distance_to_path)//geometry_msgs::Point robot_loc)
{
	geometry_msgs::Point closest_point;
    float dx,dy,dist,shortest_dist;

	for(int i = 0; i < global_plan_temp.size(); i++) 
	{
    	dx =  global_plan_temp[i].pose.position.x-robot_pose.getOrigin().getX();
		dy =  global_plan_temp[i].pose.position.y-robot_pose.getOrigin().getY();
		dist = std::sqrt(dx*dx + dy*dy);
		//std::cout << "Iterator it = " << i << endl;
		if(i==0 || dist <= shortest_dist)
		{
			closest_point.x = global_plan_temp[i].pose.position.x;
            closest_point.y = global_plan_temp[i].pose.position.y;
            closest_point.z = 0;
            shortest_dist = dist; 
            *distance_to_path = dist; // Store the shortest distance
            *plan_index = i; // Store what index in the path that is closest
		}
	}
	//std::cout << "Robot pose: " << robot_pose.getOrigin().getX() << " " << robot_pose.getOrigin().getY() << " Plan index: " << *plan_index << endl;
	return closest_point;
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
    /*for(int i=0; i<global_plan_temp.size();i++) // Look at every point on path
    {
        dx = global_plan_temp[i].pose.position.x - robot_pose.getOrigin().getX();
        dy = global_plan_temp[i].pose.position.y - robot_pose.getOrigin().getY();
        dist = std::sqrt(dx*dx + dy*dy);
        if(i==0 || dist <= shortest_dist)
        {
            closest_point.x = global_plan_temp[i].pose.position.x;
            closest_point.y = global_plan_temp[i].pose.position.y;
            closest_point.z = 0;
            shortest_dist = dist; 
            *distance_to_path = dist; // Store the shortest distance
            *plan_index = i; // Store what index in the path that is closest
        }
    }*/
/*
	for(int i = 0; i < global_plan_temp.size(); i++) 
	{
    	dx =  global_plan_temp[i].pose.position.x-robot_pose.getOrigin().getX();
		dy =  global_plan_temp[i].pose.position.y-robot_pose.getOrigin().getY();
		dist = std::sqrt(dx*dx + dy*dy);
		//std::cout << "Iterator it = " << i << endl;
		if(i==0 || dist <= shortest_dist)
		{
			closest_point.x = global_plan_temp[i].pose.position.x;
            closest_point.y = global_plan_temp[i].pose.position.y;
            closest_point.z = 0;
            shortest_dist = dist; 
            *distance_to_path = dist; // Store the shortest distance
            *plan_index = i; // Store what index in the path that is closest
		}
		i++;
	}*/
	
	
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
	/*
	for (int i = 0; i < global_plan_temp.size(); i++)
	{
		std::cout << global_plan_temp[i] << endl;
	}
	*/
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
        attractive_vector->setValue(0,0,0);
    }
}

int LocalPlanner::makeRepulsiveField(float scale, float gain, float dmax, int pos_x, int pos_y)
{
    float d, d0, d2;
    d0 = 1.0/dmax;
    //for loop
    //int current_cost = int(costmap_->getCost(getSizeInCellsX()/2,getSizeInCellsY()/2));
    // MOCK DATA
    /*int current_cost;
    if(pos_x == 40 && pos_y == 40)
        current_cost = 100;
    else 
        current_cost = 60;*/
    
    // -- MOCK DATA
	/*
	for	(int i=0; i < img_size; i++)
	{
		d = distanceClosestObstacle();
		d2 = d / scale + 1;
		d = (1 / d2 - d0)
		d = d*d;
		if (d2 <= dmax)
			current_cost = gain*d;
		else
			current_cost = 0;
	}
	*/
    int current_cost = int(costmap_->getCost(pos_x,pos_y));
    d = current_cost;
    d2 = d / scale + 1;
    d = (1 / d2 - d0);
    d = d*d;
    //std::cout << "d2: " << d2 << "dmax: " << dmax << endl;
    //if(d2 <= dmax)
    if(current_cost >= 0)
        current_cost = gain*d;
    else
        current_cost = 0;
    
    return current_cost/10000;
}

void LocalPlanner::findClosestObjectEuclidean(float pos_x, float pos_y, int *deg, float *distance_to_obstacle)
{
	//obstacle map = binary_global_map
    float min_dist = 2000.0;
    float current_dist = 0;

    int current_point = 0;

    int min_x = 0;
    int min_y = 0;

    int dist_x = 0;
    int dist_y = 0;

	//std::cout << "First";
    //for (int h = 0; h < mapHeight; h++) {
	//if (binary_global_map[pos_y / costmap_->getResolution()][pos_x / costmap_->getResolution()] == 100)
	if (binary_global_map[pos_y][pos_x] == 100)
	{
		std::cout << "Something went wrong in findClosestObjectEuclidean" << endl;
		//std::cout << "Tried to process Pos: " << pos_x / costmap_->getResolution() << " " << pos_y / costmap_->getResolution() << " But that position is an obstacle." << endl;
		std::cout << "Tried to process Pos: " << pos_x << " " << pos_y << " But that position is an obstacle." << endl;
	}
	else
	{	
		for (int h = 0; h < binary_global_map.size(); h++) {
			//for (int w = 0; w < mapWidth; w++) {
				//std::cout << "Second";
			for (int w = 0; w < binary_global_map[h].size(); w++) {
				//std::cout << "Third" << endl;
				if (binary_global_map[h][w] == 100) {
					// Calculate euclidean distance
					//std::cout << "Pos to evaluate: " << pos_x / costmap_->getResolution() << " " << pos_y / costmap_->getResolution() << endl;
					//current_dist = std::sqrt(pow(((pos_x / costmap_->getResolution()) - w),2) + pow(((pos_y/costmap_->getResolution()) - h),2));
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
	}

	min_dist = fabs(min_dist - 1);
    //dist_x = round((pos_x / costmap_->getResolution()) - min_x);
    //dist_y = round((pos_y / costmap_->getResolution()) - min_y);
	dist_x = round(pos_x - min_x);
    dist_y = round(pos_y - min_y);

    // Calculate angle
    *deg = int(round(atan2(dist_y, dist_x) * 180 / PI));

	//std::cout << "Pos: " << pos_x / costmap_->getResolution() << " " << pos_y / costmap_->getResolution() << " dist_x: " << dist_x << " dist_y: " << dist_y << endl;
	//std::cout << "Pos: " << pos_x << " " << pos_y << " dist_x: " << dist_x << " dist_y: " << dist_y << endl;    
	//std::cout << "Distance: " << min_dist << " X: " << min_x << " Y: " << min_y << " Deg: " << *deg << endl;

	//*distance_to_obstacle = min_dist * costmap_->getResolution();
	*distance_to_obstacle = min_dist;
	//std::cout << "Final distance to obstacle: " << *distance_to_obstacle << endl;
}

void LocalPlanner::makeRepulsiveFieldMatrix(float scale, float gain, float dmax, float pos_x, float pos_y, float *repulsive_force, int *deg)
{
    float d, d0, d2, distance_to_obstacle;
	//std::cout << "Pos to send: " << pos_x << " " << pos_y << endl;
	findClosestObjectEuclidean(pos_x, pos_y, deg, &distance_to_obstacle);
	
	d0 = 1.0/dmax;
	d = distance_to_obstacle;
	d2 = d / scale + 1;
	d = (1 / d2 - d0);
	d = d*d;
	if (d2 <= dmax)
		*repulsive_force = gain*d;
	else
		*repulsive_force = 0;
	
	//if(pos_x == 40)
		//std::cout << "Pos: " << pos_x << " " << pos_y << " d2: " << d2 << " dmax: " << dmax << endl;

    //int current_cost = int(costmap_->getCost(pos_x,pos_y));
    //d = current_cost;
    //d2 = d / scale + 1;
    //d = (1 / d2 - d0);
    //d = d*d;
    //std::cout << "d2: " << d2 << "dmax: " << dmax << endl;
    //if(d2 <= dmax)
    //if(current_cost >= 0)
        //current_cost = gain*d;
   // else
        //current_cost = 0;
    
    //return current_cost/10000;
}

void LocalPlanner::repulsiveForceMatrix(tf::Stamped<tf::Pose> robot_pose, tf::Vector3 *repulsive_vector)
{
	// Robot size has a radius of 0.2m in simulation with center at middle.
	// In the paper, dmax = 1.25, radius = 0.5m, resolution = 0.1
	// dmax = 1 + radius / 2
	// If radius = 0.2, gives dmax = 1+0.2/2 = 1.1;
    int scale = 100;
    int gain = 10000;
    //float dmax = 1.25; // Default
	// Every 0.1 is about 50cm in a resolution of 0.05
	float dmax = 1.01; 
	//float dmax = 1.003; // 1.003 is about 10 spots away
	float repulsive_force, x_force, y_force, z_force;
	int deg;
    //xt = robot_pose.getOrigin().getX();
    //yt = robot_pose.getOrigin().getY();
    //std::cout << "Coordinate" << costmap_->getSizeInCellsX()/2 + 1 << endl;
    // Mock Data
        //int getSizeInCellsX = 40;
        //int getSizeInCellsY = 40;
    //
    makeRepulsiveFieldMatrix(scale, gain, dmax, robot_pose.getOrigin().getX(), robot_pose.getOrigin().getY(), &repulsive_force, &deg);
	//std::cout << "Pos: " << robot_pose.getOrigin().getX() << " " << robot_pose.getOrigin().getY() << " Force: " << repulsive_force << " At degree: " << deg << endl;

	if (repulsive_force > 0)
	{
		y_force = repulsive_force*sin(deg*PI/180);
		x_force = repulsive_force*cos(deg*PI/180);
		z_force = 0;
		repulsive_vector->setValue(x_force,y_force,z_force);
		//repulsive_vector->normalize();
		//repulsive_vector->operator*=(scale);
	}
	else
	{
		repulsive_vector->setValue(0,0,0);
	}
	

	/*if(robot_pose.getOrigin().getY() == 55)
	{
		std::cout << "Pos: " << robot_pose.getOrigin().getX() << " " << robot_pose.getOrigin().getY() << " Force: " << repulsive_force << " At degree: " << deg << endl;
		std::cout << "Gives a vector: " << repulsive_vector->m_floats[0] << " " << repulsive_vector->m_floats[1] << " " << repulsive_vector->m_floats[2] << endl;
	}*/

	//repulsive_vector->setValue(0,0,0);
}

void LocalPlanner::repulsiveForce(tf::Stamped<tf::Pose> robot_pose, tf::Vector3 *repulsive_vector)
{
    int xt, yt, cost, force_x, force_y;
    int scale = 100;
    int gain = 10000;
    float dmax = 1.25;
    //std::cout << "Coordinate" << costmap_->getSizeInCellsX()/2 + 1 << endl;
    // Mock Data
        //int getSizeInCellsX = 40;
        //int getSizeInCellsY = 40;
    //
    //force_x = makeRepulsiveField(scale, gain, dmax, robot_pose.getOrigin().getX(), robot_pose.getOrigin().getY());
    force_x = - makeRepulsiveField(scale, gain, dmax, costmap_->getSizeInCellsX()/2 + 1, costmap_->getSizeInCellsY()/2) + makeRepulsiveField(scale, gain, dmax, costmap_->getSizeInCellsX()/2, costmap_->getSizeInCellsY()/2);
    force_y = - makeRepulsiveField(scale, gain, dmax, costmap_->getSizeInCellsX()/2, costmap_->getSizeInCellsY()/2 + 1) + makeRepulsiveField(scale, gain, dmax, costmap_->getSizeInCellsX()/2, costmap_->getSizeInCellsY()/2);
	//force_x = - makeRepulsiveField(scale, gain, dmax, getSizeInCellsX + 1, getSizeInCellsY) + makeRepulsiveField(scale, gain, dmax, getSizeInCellsX, getSizeInCellsY);
    //force_y = - makeRepulsiveField(scale, gain, dmax, getSizeInCellsX, getSizeInCellsY + 1) + makeRepulsiveField(scale, gain, dmax, getSizeInCellsX, getSizeInCellsY);
    repulsive_vector->setValue(force_x,force_y,0);
}

void LocalPlanner::updateVelocity(tf::Vector3 force, tf::Stamped<tf::Pose> robot_pose, float *linear_velocity, float *angular_velocity) 
{
	float u, omega;
	float k_omega = k2;
	float k_u = k1;
	float pos_x = robot_pose.getOrigin().getX();
	float pos_y = robot_pose.getOrigin().getY();
	//float theta = robot_pose.getOrigin().getYaw();
	float theta = tf::getYaw(robot_pose.getRotation());
	float goal_x = global_goal_odom.getOrigin().getX();
	float goal_y = global_goal_odom.getOrigin().getY();
	float d = (pos_x - goal_x)*(pos_x - goal_x) + (pos_y - goal_y)*(pos_y - goal_y);
	d = sqrt(d);// / 20;
	u = 20*k_u*tanh(d);
	omega = -k_omega*(theta - atan2(force.m_floats[1], force.m_floats[0]));
	//pos.x = pos.x + (int)(u*DELTA_T*cos(theta));
	//pos.y = pos.y + (int)(u*DELTA_T*sin(theta));
	//theta = theta + omega*DELTA_T;
	*linear_velocity = u;//Point(u*cos(theta), u*sin(theta));
	*angular_velocity = omega;
	std::cout << "Theta: " << theta << " omega: " << omega << endl;
}

void LocalPlanner::staticFlowFieldCalcMatrix(void)
{
	tf::Stamped<tf::Pose> robot_pose;
	geometry_msgs::Point fa_point_path;
	tf::Vector3 f_attractive_vector, unit_vector_ni, f_repulsive_vector, final_vector;
	vector<tf::Vector3> temp_matrix;
	int plan_index = 0;
	float distance_to_path;
	//std::cout << "First" << endl;
	total_static_flow_field.clear();
	//std::cout << "Binary global map = " << binary_global_map.size() << " * " << binary_global_map[0].size() << endl;
	for (int i = 0; i < binary_global_map.size(); i++)
	{
		//std::cout << "First loop" << endl;
		for (int j = 0; j < binary_global_map[i].size(); j++)
		{
			//std::cout << "Second Loop" << endl;
			if (binary_global_map[i][j] != 100)
			{
				if(binary_global_map[i][j] == 100)
					std::cout << "ERROR" << endl;
				//std::cout << "If state" << endl;
				robot_pose.getOrigin().setX(j*costmap_->getResolution()); // Sets the robot position in meters
				robot_pose.getOrigin().setY(i*costmap_->getResolution());
				//std::cout << "Peter 2" << endl;
				fa_point_path = closestPointOnPathMatrix(robot_pose, &plan_index, &distance_to_path);
				//if(robot_pose.getOrigin().getY() < 4)
					//std::cout << "Pos: " << robot_pose.getOrigin().getX() << " " << robot_pose.getOrigin().getY() << " plan index " << plan_index << " Point on path: " <<  fa_point_path.x << " " << fa_point_path.y << endl;

				if (distance_to_path > 100)
				{
					// Re-calculate Thetastar
				}
				//std::cout << "Peter 4" << endl;
				unit_vector_ni = unitVectorOfPath(plan_index);
				//if(unit_vector_ni.m_floats[0] == 0)
					//std::cout << "Pos: " << robot_pose.getOrigin().getX() << " " << robot_pose.getOrigin().getY() << " vector " << unit_vector_ni.m_floats[0] << " " << unit_vector_ni.m_floats[1] << " " << unit_vector_ni.m_floats[2] << endl;
				//std::cout << "Peter" << endl;
				fAttractiveVector(fa_point_path, robot_pose, unit_vector_ni, &f_attractive_vector);
				//if(isnan(unit_vector_ni.m_floats[0]))
					//std::cout << "Pos: " << robot_pose.getOrigin().getX() << " " << robot_pose.getOrigin().getY() << " NAN Value " << f_attractive_vector.m_floats[0] << endl;
				//std::cout << "Peter 5" << endl;
				f_attractive_vector.operator*=(1 - exp(-k1*distance_to_path)); 
				//std::cout << "Peter 6" << endl;
				f_attractive_vector.operator+=(unit_vector_ni.operator*=(k2*exp(-k1*distance_to_path)));
				//std::cout << "Robot position: " << robot_pose.getOrigin().getX() << " " << robot_pose.getOrigin().getY() << endl;
				//std::cout << "Closest point on path: " << plan_index << " distance to path: " << distance_to_path << endl;
				//std::cout << "Vector: " << f_attractive_vector.m_floats[0] << " " << f_attractive_vector.m_floats[1] << " " << f_attractive_vector.m_floats[2] << endl;
				robot_pose.getOrigin().setX(j); // Sets the robot position in matrix
				robot_pose.getOrigin().setY(i);
				repulsiveForceMatrix(robot_pose, &f_repulsive_vector);
				/*if(robot_pose.getOrigin().getX() == 20)
				{
					tf::Vector3 potential_final_vector;
					std::cout << "Pos: " << robot_pose.getOrigin().getX() << " " << robot_pose.getOrigin().getY() << endl;
					std::cout << "Gives a repulsive vector: " << f_repulsive_vector.m_floats[0] << " " << f_repulsive_vector.m_floats[1] << endl;
					std::cout << "Gives an attractive vector: " << f_attractive_vector.m_floats[0] << " " << f_attractive_vector.m_floats[1] << endl;
					potential_final_vector = f_attractive_vector;
					potential_final_vector.operator+=(f_repulsive_vector);
					potential_final_vector.normalize();
					std::cout << "Potential vector would be: " << potential_final_vector.m_floats[0] << " " << potential_final_vector.m_floats[1] << endl;
				}*/
				final_vector = f_attractive_vector;
				//final_vector = f_repulsive_vector;
				//if(isnan(f_attractive_vector.m_floats[0]))
				//	std::cout << "Pos: " << robot_pose.getOrigin().getX() << " " << robot_pose.getOrigin().getY() << " NAN Value " << f_attractive_vector.m_floats[0] << endl;
				final_vector.operator+=(f_repulsive_vector);
				final_vector.normalize();
				if(i == 3)
					std::cout << "Repulsive Vector at y == 3: " << final_vector.m_floats[0] << " " << final_vector.m_floats[1] << endl;

			}
			else
			{
				//std::cout << "Else state" << endl;
				final_vector.setValue(0,0,0);
			}
			//std::cout << "Before push back" << endl;
			temp_matrix.push_back(final_vector);
			//std::cout << "After push back" << endl;
			//std::cout << temp_matrix[0].m_floats[0];
		}
		total_static_flow_field.push_back(temp_matrix);
		temp_matrix.clear();
	}
	//std::cout << "Second" << endl;
	for (int i=0; i<total_static_flow_field.size(); i++)
	{
		for (int j=0; j<total_static_flow_field[i].size(); j++)
		{
			//if(total_static_flow_field[i][j].m_floats[0] == 0 && total_static_flow_field[i][j].m_floats[1] == 0)
			//	std::cout << "Vector at : " << j << ", " << i << " :" << total_static_flow_field[i][j].m_floats[0] << " " << total_static_flow_field[i][j].m_floats[1] << " " << total_static_flow_field[i][j].m_floats[2] << endl;

			if(i == 3)
				std::cout << "Vector at y == 2: " << total_static_flow_field[i][j].m_floats[0] << " " << total_static_flow_field[i][j].m_floats[1] << " " << total_static_flow_field[i][j].m_floats[2] << endl;
			//if(!binary_global_map[i][j] == 0)
				//std::cout << " ";
			//std::cout << total_static_flow_field[i][j].m_floats[0];
			//if(!binary_global_map[i][j] == 0)
				//std::cout << " ";
		}
		//std::cout << endl;
	}
	/*if(isnan(total_static_flow_field[0][0].m_floats[0]))
    {
		std::cout << "NAN" << endl;
        //attractive_vector->setValue(0,0,0);
    }*/
	//std::cout << total_static_flow_field[0][0].m_floats[0];
	//std::cout << temp_matrix[0].m_floats[0];
	//std::cout << "Flow field size = " << total_static_flow_field.size() << " * " << total_static_flow_field[0].size() << endl;

	//robot_pose.getOrigin().setX(5);
	//robot_pose.getOrigin().setY(5);
	//fa_point_path = closestPointOnPathMatrix(robot_pose, &plan_index, &distance_to_path);
	//std::cout << "Last point: " << fa_point_path.x << ", " << fa_point_path.y << endl;
}

void LocalPlanner::getPositionInMatrix(tf::Stamped<tf::Pose> robot_pose, int *coord_x, int *coord_y)
{
	tf::Stamped<tf::Pose> world_point;
	tf_.transformPose("/map", robot_pose, world_point);

	//std::cout << "World point in map: " << world_point.getOrigin().getX() << " " << world_point.getOrigin().getY() << endl;

	//float size_meters_x = global_map_cells_x / costmap_->getResolution();
	//float size_meters_y = global_map_cells_y / costmap_->getResolution();

	*coord_x = round(world_point.getOrigin().getX() / costmap_->getResolution());
	*coord_y = round(world_point.getOrigin().getY() / costmap_->getResolution());

	//std::cout << "Coordinates in matrix: " << *coord_x << " " << *coord_y << endl;
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
    tf::Vector3 f_attractive_vector;
    tf::Vector3 f_repulsive_vector;
	tf::Vector3 final_vector;

	getPositionInMatrix(robot_pose, &coord_x, &coord_y);
	final_vector = total_static_flow_field[coord_y][coord_x];
	//final_vector = total_static_flow_field[180][180];

	/*for(int i=0; i<binary_global_map.size(); i++)
	{
		std::cout << binary_global_map[i][100] << " ";
	}
	std::cout << endl;
	*/
	//std::cout << "Binary global map [0][0]: " << binary_global_map[50][1] << endl;

	//global_plan_temp = global_plan_;

	/*for(int i = 0; i < global_plan_temp.size(); i++)
	{
		std::cout << "plan coordinate: " << global_plan_temp[i].pose.position.x << endl;
	}*/
	
    /*geometry_msgs::Point fa_point_path = closestPointOnPath(robot_pose, &plan_index, &distance_to_path);
    if (distance_to_path > 100)
    {
        // Re-calculate Thetastar
    }
	//std::cout << "Size of global plan: " << global_plan_temp.size() << endl;
	//std::cout << "closest index on path: " << plan_index << " Distance to path: " << distance_to_path << endl;
    //std::cout << "Robot's position x:" << robot_pose.getOrigin().getX() << " y:" << robot_pose.getOrigin().getY() << endl;
	//std::cout << "Position to reach x:" << fa_point_path.x << " y:" << fa_point_path.y << endl;
    //std::cout << "plan_index = " << plan_index << endl;
    tf::Vector3 unit_vector_ni(unitVectorOfPath(plan_index));

    //std::cout << "Distance to path = " << distance_to_path << endl;
    //std::cout << "Line segment heading = " << unit_vector_ni.m_floats[0] << " " << unit_vector_ni.m_floats[1] << " " << unit_vector_ni.m_floats[2] << endl;

    fAttractiveVector(fa_point_path, robot_pose, unit_vector_ni, &f_attractive_vector);
	//std::cout << "First attractive vector " << f_attractive_vector.m_floats[0] << " " << f_attractive_vector.m_floats[1] << " " << f_attractive_vector.m_floats[2] << endl;
    f_attractive_vector.operator*=(1 - exp(-k1*distance_to_path)); 
	//std::cout << "Second attractive vector " << f_attractive_vector.m_floats[0] << " " << f_attractive_vector.m_floats[1] << " " << f_attractive_vector.m_floats[2] << endl;
    f_attractive_vector.operator+=(unit_vector_ni.operator*=(k2*exp(-k1*distance_to_path)));
    //std::cout << "Attractive vector " << f_attractive_vector.m_floats[0] << " " << f_attractive_vector.m_floats[1] << " " << f_attractive_vector.m_floats[2] << endl;
    repulsiveForce(robot_pose, &f_repulsive_vector);

    //std::cout << "Repulsive vector " << f_repulsive_vector.m_floats[0] << " " << f_repulsive_vector.m_floats[1] << " " << f_repulsive_vector.m_floats[2] << endl;
	final_vector = f_attractive_vector.operator+=(f_repulsive_vector);
	std::cout << "Final vector: " << final_vector.m_floats[0] << " " << final_vector.m_floats[1] << " " << final_vector.m_floats[2] << endl;
	updateVelocity(final_vector, robot_pose, &linear_velocity, &angular_velocity);
	
	//std::cout << "Linear = " << linear_velocity << " Angular = " << angular_velocity << endl;
	cmd_vel.angular.z = angular_velocity;
	cmd_vel.linear.x = linear_velocity;
	*/
	// -- Peter

	/*for (int i = 0; i < footprint_spec_.size(); i++)
	{
		std::cout << "Footprint nr: " << i << " = " << endl;//footprint_spec_[i] << endl;
	}*/
	

	std::cout << "Final vector: " << final_vector.m_floats[0] << " " << final_vector.m_floats[1] << " " << final_vector.m_floats[2] << " At position: " << coord_x << " " << coord_y << endl;
	updateVelocity(final_vector, robot_pose, &linear_velocity, &angular_velocity);
	cmd_vel.angular.z = angular_velocity;
	cmd_vel.linear.x = linear_velocity;
	minCost = 255;//std::numeric_limits<double>::max();

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
		generate_new_plan = 1;
		return true;

	}
	
   	return false;
}

// Added by Peter
//void mapChatterCallback(costmap_2d::Costmap2DROS *costmap_ros)
void LocalPlanner::mapChatterCallback(const nav_msgs::OccupancyGrid msg)
{
  local_occupancy_grid = msg;
}
// -- Peter

bool LocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
   	if(!initialized_)
	{
		ROS_ERROR("Please call initialize() before using this planner");
		return false;
	}

	global_plan_.clear();
	global_plan_temp.clear();
	global_plan_ = plan;
	global_plan_temp = global_plan_;

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

	ros::NodeHandle test_node_peter;
	LocalPlanner map_listener;
	if (generate_new_plan == 1)
	{
		do{
			test_subscriber = test_node_peter.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &LocalPlanner::mapChatterCallback, &map_listener);
			ros::spinOnce();
			//ros::spin();
			if (!map_listener.local_occupancy_grid.info.width == 0)
			{
				binary_global_map.clear();
				//std::cout << "Map info: " << map_listener.local_occupancy_grid.info.width << " " << map_listener.local_occupancy_grid.info.height << endl;
				for (int i=0; i < map_listener.local_occupancy_grid.info.height; i++)
				{
					vector<int> temp_matrix;
					for (int j=0; j < map_listener.local_occupancy_grid.info.width; j++)
					{
						temp_matrix.push_back(map_listener.local_occupancy_grid.data[i*map_listener.local_occupancy_grid.info.height+j]);
					}
					binary_global_map.push_back(temp_matrix);
				}
				/*for (int i=0; i<binary_global_map.size();i++)
				{
					for (int j=0; j<binary_global_map[i].size();j++)
					{
						if(!binary_global_map[i][j] == 0)
							std::cout << " ";
						std::cout << binary_global_map[i][j];
						if(!binary_global_map[i][j] == 0)
							std::cout << " ";
					}
					std::cout << endl;
				}*/
				//global_plan_temp = global_plan_;
				/*int dx, dy;
				std::vector<geometry_msgs::PoseStamped> transformed_global_plan;
				if(base_local_planner::transformGlobalPlan(tf_,global_plan_temp,robot_pose,*costmap_,global_frame_,transformed_global_plan)) //Transforme the plan respect to the robot base
				{
					int i = 0;
					for(std::vector<geometry_msgs::PoseStamped>::iterator it = transformed_global_plan.begin(); it != transformed_global_plan.end(); ++it) 
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
					for (int i=0; i<transformed_global_plan.size(); i++)
					{
						std::cout << "Transformed global plan: " << transformed_global_plan[i].pose.position.x << " " << transformed_global_plan[i].pose.position.y << endl;
					}*/
					//for (int i=0; i<global_plan_.size(); i++)
					//{
						//std::cout << "Transformed global plan: " << global_plan_[i].pose.position.x << " " << global_plan_[i].pose.position.y << endl;
					//}
				//}
				std::cout << "map updated" << endl;
				staticFlowFieldCalcMatrix();
			}
		} while(map_listener.local_occupancy_grid.info.width == 0);
	}
	generate_new_plan = 0;
	global_map_cells_x = local_occupancy_grid.info.width;
	global_map_cells_y = local_occupancy_grid.info.width;
	//std::cout << "setPlan done" << endl;

	  //std::cout << endl;
	//binary_global_map = map_listener
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
	generate_new_plan = 1;
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

 /*----------------------------------------------*/

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
