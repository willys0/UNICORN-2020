/** include the libraries you need in your planner here */
 /** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
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

//Added by Peter
#include <costmap_2d/layered_costmap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>
#include <vector_creation/vectors.h>
#include <vector_creation/vector.h>
//#include "vectors.h"
//#include "vector.h"
//#include "../../../../devel/include/vector_creation/vectors.h"
//#include "vector_creation/vector.h"
//#include <costmap_2d>
// -- Peter

 using std::string;

 #ifndef LOCAL_PLANNER_CPP
 #define LOCAL_PLANNER_CPP

#define MAX_LINEAR_VEL 0.5
#define MIN_LINEAR_VEL 0.0
#define MAX_ANGULAR_VEL 0.5
#define MIN_ANGULAR_VEL -0.5
#define GAMA 1.0
#define PI 3.14159265358979323846264338327950288

#define MAX_ACC_LINEAR 1.0
#define MAX_ACC_ANGULAR 2.0

#define DT 0.10000 //It defines the granularity to create a trajectory. A higher value will create a few points on a trajectory.


 namespace local_planner {

 class LocalPlanner : public nav_core::BaseLocalPlanner {
 public:

	LocalPlanner();
	LocalPlanner(std::string name,  tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);

	/** overridden classes from interface nav_core::BaseLocalPlanner **/
	bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
	bool isGoalReached();
	bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
	void initialize (std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros) ;
void dynamicObstacleCallback(const vector_creation::vector msg);
//static void chatterCallback(const std_msgs::String::ConstPtr& msg);

  private:

	// Made by Peter
geometry_msgs::Point closestPointOnPath(tf::Stamped<tf::Pose> robot_pose, int *plan_index, float *distance_to_path);
tf::Vector3 unitVectorOfPath(int plan_index);
void fAttractiveVector(geometry_msgs::Point fa_point_path, tf::Stamped<tf::Pose> robot_pose, tf::Vector3 unit_vector_ni, tf::Vector3 *attractive_vector);
void makeRepulsiveField(int scale, int gain, float dmax, float pos_x, float pos_y, float *repulsive_force, int *deg);
void repulsiveForce(tf::Stamped<tf::Pose> robot_pose, tf::Vector3 *repulsive_vector);
void updateVelocity(tf::Vector3 force, tf::Stamped<tf::Pose> robot_pose, float *linear_velocity, float *angular_velocity, double repulsive_field_magnitude, double dipole_field_magnitude, float distance_to_path);
void findClosestObjectEuclidean(int *deg, float *distance_to_obstacle);
void dipoleForce(tf::Vector3 robot_moment, tf::Stamped<tf::Pose> robot_pose, tf::Vector3 *dipole_force);
//static const float k1 = 0.01;
//static const float k2 = 1;
float k1;
float k2;
static const float DELTA_T = 1.0;
int test_variable;
int generate_new_path;
float last_linear_velocity_;
static const float max_linear_vel = 0.4; // default 0.4
static const float max_angular_vel = 0.4;
static const float max_linear_acc_ = 0.05;
vector_creation::vector obstacle_vector_;
bool close_to_goal;
std::vector<tf::Vector3> final_vectors_vector;
double last_repulsive_field_magnitude_;
	// -- Peter

  	costmap_2d::Costmap2DROS* costmap_ros_; //!< Pointer to the costmap ros wrapper, received from the navigation stack
  	costmap_2d::Costmap2D* costmap_; //!< Pointer to the 2d costmap (obtained from the costmap ros wrapper)
  	tf::TransformListener tf_; //!< pointer to Transform Listener
 
	std::vector<geometry_msgs::PoseStamped> global_plan_; //!< Store the current global plan
	std::vector<geometry_msgs::PoseStamped> global_plan_temp; //!< Store the current global plan
	bool goal_reached_; //!< store whether the goal is reached or not
 
	std::string global_frame_; //!< The frame in which the controller will run
	std::string robot_base_frame_; //!< Used as the base frame id of the robot

	// flags
	bool initialized_; //!< Keeps track about the correct initialization of this class
	
	ros::Publisher l_plan_pub_,marker_pub;
	ros::Subscriber odom_sub_,moving_obstacle_subscriber_;

	uint32_t shape; //Marker to represent local goal


	std::vector<geometry_msgs::Point> footprint_spec_;

	tf::Stamped<tf::Pose> global_goal,global_goal_odom; 
    

  };
 };
 #endif


