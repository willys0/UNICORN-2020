/**	
*	@file local_planner.h
*/

#ifndef LOCAL_PLANNER_CPP
#define LOCAL_PLANNER_CPP

/*ROS*/
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <costmap_2d/layered_costmap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <vector_creation/vectors.h>
#include <vector_creation/vector.h>
#include <std_msgs/Int32.h>

 using std::string;



#define MAX_LINEAR_VEL 0.4
#define MIN_LINEAR_VEL 0.05
#define MAX_ANGULAR_VEL 0.4
#define MIN_ANGULAR_VEL -0.4
#define GAMA 1.0
#define PI 3.14159265358979323846264338327950288

#define MAX_ACC_LINEAR 0.01
#define MAX_ACC_ANGULAR 2.0

#define DT 0.10000 //It defines the granularity to create a trajectory. A higher value will create a few points on a trajectory.
#define DELTA_T 1.0

#define DIPOLE_FIELD_THRESHOLD 0.5
#define REPULSIVE_FIELD_THRESHOLD 0.5


 namespace local_planner {
/** @brief Class for the local planner in the move_base node.
*
*	
*		
*/
 class LocalPlanner : public nav_core::BaseLocalPlanner {
 public:
	/*Members*/

	/*Methods*/
	/**
	* @brief local_planner class Constructor.
	* @param name Name of the local planner.
	* @param tf Pointer to the transform.
	* @param costmap_rosname Cost map to use.
	*/
	LocalPlanner(std::string name,  tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);
	/**
	* @brief Default constructor.
	*/
	LocalPlanner();
	/**
	* @brief Method that calculates everything about the robot's movement.
	* @param cmd_vel Message with velocity commands to send to the robot base.
	* @return True if a valid velocity was sent.
	*/
	bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
	/**
	* @brief Method which checks if the robot has reached its goal. If true, it also sends the current position to the global planner to see if it was just a milestone or not.
	* @return True if goal reached.
	*/
	bool isGoalReached();
	/**
	* @brief Method that receives the path from the global planner
	* @param plan Includes positions along the path
	* @return True if a plan was received.
	*/
	bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
	/**
	* @brief Constructs the local planner
	* @param name Name of the local planner
	* @param tf Pointer to the transform
	* @param costmap_rosname Cost map to use
	*/
	void initialize (std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros) ;
	/**
	* @brief Callback method that listens on a topic for eventual dynamic obstacles.
	* @param msg Includes information about the obstacle(s).
	*/
void dynamicObstacleCallback(const vector_creation::vector msg);

  private:
	/*Methods*/
	/**
	* @brief Method for the dipole flow field algorithm. It checks the closest point on path, relative the robot position.
	* @param robot_pose Current pose of the robot.
	* @param plan_index Variable to see the index in the plan of the closest point.
	* @param distance_to_path Containts the distance to the closest point.
	* @return The coordinate of the closest point.
	*/
	geometry_msgs::Point closestPointOnPath(tf::Stamped<tf::Pose> robot_pose, int *plan_index, float *distance_to_path);
	/**
	* @brief Method for the dipole flow field algorithm. It calculates a unit vector along the path, from the closest point on path.
	* @param plan_index Index of the closest point on path to "start" from.
	* @return The calculated vector.
	*/
	tf::Vector3 unitVectorOfPath(int plan_index);
	/**
	* @brief Method for the dipole flow field algorithm. It calculates the attractive force.
	* @param fa_point_path Variable for the attractive force algorithm.
	* @param robot_pose Pose of the robot.
	* @param unit_vector_ni Variable for the attractive force algorithm.
	* @param attractive_vector Stores the attractive force vector.
	*/
	void fAttractiveVector(geometry_msgs::Point fa_point_path, tf::Stamped<tf::Pose> robot_pose, tf::Vector3 unit_vector_ni, tf::Vector3 *attractive_vector);
	/**
	* @brief Method for the dipole flow field algorithm. It computes the magnitude of the repulsive force.
	* @param scale Variable for the dipole flow field algorithm.
	* @param gain Variable for the dipole flow field algorithm.
	* @param dmax Variable for the dipole flow field algorithm.
	* @param pos_x Robot's x position.
	* @param pos_y Robot's y position.
	* @param repulsive_force Holds the magnitude of the repulsive force.
	* @param deg Holds the degree to an obstacle, relative the robot.
	*/
	void makeRepulsiveField(int scale, int gain, float dmax, float pos_x, float pos_y, float *repulsive_force, int *deg);
	/**
	* @brief Method for the dipole flow field algorithm. It computes the repulsive force vector.
	* @param robot_pose Pose of the robot.
	* @param repulsive_vector Holds the repulsive force vector.
	*/
	void repulsiveForce(tf::Stamped<tf::Pose> robot_pose, tf::Vector3 *repulsive_vector);
	/**
	* @brief Method that calculates the velocities for the robot, i.e. this is the velocity control function. It includes some calculations from the dipole flow field algorithm. It does also eventually modify a variable for the attractive force as well as sending the robot's desired heading for the "human robot interaction" method.
	* @param force The final force, from the dipole flow field algorithm, to act upon.
	* @param robot_pose Pose of the robot.
	* @param linear_velocity This variable is updated with the desired linear velocity.
	* @param angular_velocity This variable is updated with the desired angular velocity.
	* @param repulsive_field_magnitude Magnitude of the repulsive force vector.
	* @param dipole_field_magnitude Magnitude of the dynamic dipole force vector.
	* @param distance_to_path Distance to path.
	*/
	void updateVelocity(tf::Vector3 force, tf::Stamped<tf::Pose> robot_pose, float *linear_velocity, float *angular_velocity, double repulsive_field_magnitude, double dipole_field_magnitude, float distance_to_path);
	/**
	* @brief Method for the dipole flow field algorithm. It calculates the distance and angle to the closest obstacle, relative the robot position.
	* @param deg Holds the angle, in degrees, to the closest obstacle.
	* @param distance_to_obstacle Holds the distance to the closest obstacle.
	*/
	void findClosestObjectEuclidean(int *deg, float *distance_to_obstacle);
	/**
	* @brief Method for the dipole flow field algorithm. It calculates the dynamic dipole force for the robot.
	* @param robot_moment Vector where the robot wants to go.
	* @param robot_pose Pose of the robot.
	* @param dipole_force Holds the vector of the dynamic dipole force.
	*/
	void dipoleForce(tf::Vector3 robot_moment, tf::Stamped<tf::Pose> robot_pose, tf::Vector3 *dipole_force);
	/*Members*/
	float k1; //!< Variable for the attractive force
	float k2; //!< Variable for the attractive force
	int test_variable; //!< Mock data for the dipoleForce method.
	int generate_new_path; //!< Variable to tell the local planner to receive a new path.
	float last_linear_velocity_; //!< Holds the linear velocity from the previous time instance.
	vector_creation::vector obstacle_vector_; //!< Holds the information of dynamic obstacles in sight.
	bool close_to_goal; //!< Tells the velocity control function if the robot is close to the goal or not. If true the robot will have zero linear velocity, i.e. it will only adjust its heading.
	std::vector<tf::Vector3> final_vectors_vector; //!< Currenty not in use
	double last_repulsive_field_magnitude_; //!< Holds the previous repulsive force magnitude. Used in the velocity control function.
	float shortest_distance_to_obstacle_; //!< Stores the distance to the closest obstacle. Used when deciding wheter a new path should be obtained or not.
	int deg_to_obstacle_; //!< Angle in degrees to the closest obstacle.
	float last_k2_; //!< Holds the previous k2 value. Used in attractive force calculation and also eventually modified in the velocity control function.
  	costmap_2d::Costmap2DROS* costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack.
  	costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper).
  	tf::TransformListener tf_; //!< pointer to Transform Listener. 
	std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan.
	std::vector<geometry_msgs::PoseStamped> global_plan_temp; //!< Store the current global. plan
	bool goal_reached_; //!< store whether the goal is reached or not. 
	std::string global_frame_; //!< The frame in which the controller will run.
	std::string robot_base_frame_; //!< Used as the base frame id of the robot.
	bool initialized_; //!< Keeps track about the correct initialization of this class.	
	ros::Publisher l_plan_pub_,marker_pub,new_map_pub,intention_pub,position_pub; //!< ROS publishers.
	ros::Subscriber odom_sub_,moving_obstacle_subscriber_; //!< ROS subscribers.
	uint32_t shape; //!< Currently not in use. Was to show the magnetic field.
	std::vector<geometry_msgs::Point> footprint_spec_; //!< Currently not in use
	tf::Stamped<tf::Pose> global_goal,global_goal_odom; //!< Stores the goal.
    

  };
 };
 #endif // !ST


