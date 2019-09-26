/** 
*	@file unicorn_statemachine.h
*	@ Original Author Alexander Karlsson (akn13013@student.mdh.se)
	@ Build upon by Billy Lindgren (bln13004@student.mdh.se)
*/

#ifndef UNICORN_STATEMACHINE_H
#define UNICORN_STATEMACHINE_H

/* ROS */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/Range.h>
#include <unicorn/CharlieCmd.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
/* C / C++ */
#include <iostream>
#include <termios.h>
#include <cmath>
#include <boost/lexical_cast.hpp>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; /**< Client that calls actions from move_base */

/** @brief Function to get sign of input. */
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}



/** Current state of the robot. */
namespace current_state
{
	enum
	{
		AUTONOMOUS,
		MANUAL,
		LOADING,
		IDLE,
		ALIGNING,
		EXITING,
		ENTERING,
		LIFT,
		REVERSING
	};
}

/** @brief Storage class for refuse bin position*/
class RefuseBin
{
public:
	RefuseBin();
	float x;
	float y;
	float yaw;
};

/** @brief Class to define a PID controller.
*	
*	Controls a float variable based on input error.	
*/
class PidController
{
public:
	/** @brief Constructor to initialize PID gains and error tolerance.
	*	
	*	@param Kp 	Proportional gain.
	*	@param Ki 	Integral gain.
	*	@param Kd 	Derivative gain.
	*	@param tolerance 	error tolerance.
	*/
	PidController(float Kp, float Ki, float Kd, float tolerance);
	/** @brief Calculate pidterm and control variable
	*	
	*	@param error 	error of output response
	*	@param var 	new input into pid loop	
	*/
	void control(float& var, float error);
	void setLimit(double lower, double upper);
	float limit(float term);
private:
	const float Kp_, Ki_, Kd_, tolerance_;
	float previous_error_;
	float total_error_;
	double lower_limit_, upper_limit_;
};

class RangeSensor
{
public:
	RangeSensor(const std::string& sensor_topic);
	const std::string TOPIC;
	float getRange();
	void rangeCallback(const sensor_msgs::Range& msg);
private:
	ros::NodeHandle n_;
	ros::Subscriber range_sub_;

	float range_;	
};

/** @brief Main node class for unicorn_statemachine.
*
* C++ API for sending commands to the robot 
* during the debugging process.
*/
class UnicornState
{
public:
	/** @brief Initializes publishers/subscribers. */
	UnicornState();
	/** @brief Start global localization.
	*
	* Calls rosservice /global_localization
	* from rosnode /amcl.
	*/
	void globalLocalization();
	/** @brief Non-blocking get char.*/
	int getCharacter();
	/** @brief Calls functions from keyboard input.
	*
	* @param c input key.
	*/
	void processKey(int c);
	int getInput(float& val);
	/** @brief Prints key usage.*/
	void printUsage();
	/** @brief Outer loop.*/
	void active();
	/** @brief Maps current state to a string.
	*
	* @param state current machine state.
	* @return string state as string.
	*/
	std::string stateToString(int state);
	void odomCallback(const nav_msgs::Odometry& msg);
	void rangeCallback(const sensor_msgs::Range& msg);
	void bumperCallback(const std_msgs::Bool& pushed_msg);
  void LiftCallback (const std_msgs::Int8& recieveMsg); // ADDED BY MUJI
	/** @brief Sends a goal on the map to move_base.
	*
	* @param x,y target point on map.
	* @param yaw target heading.
	*/
	int sendGoal(const float& x, float y, float yaw);
	/** @brief Sends a goal to move_base.
	*
	* The goal is relative to current robot position.
	* @param x,y target point on map.
	* @param yaw target heading.
	*/
	void sendMoveCmd(float x, float y, float yaw);
	/** @brief Cancels all current goals.*/
	void cancelGoal();


	void giveOrder();
	/** @brief sends predefined orders */

	void reverse();
	/** @brief the agent reverse until the bumper is pressed  */

	void lift();
	/** @brief check if the lift is up and down, and send command to arduino to either raise or lower the lift */
	
	bool accGoalServer(unicorn::CharlieCmd::Request  &req,
         unicorn::CharlieCmd::Response &res);
	
	/* Functions below this comment section are for testing purposes*/
	void clicked_PointCallback(const geometry_msgs::PointStamped& msg); // this functions needs to subscribe to topic /clicked_point, which gets generated in rviz
	void rvizWPmaker(double posX, double posY);
	void pathCreator();
private:
	ros::NodeHandle n_;
	ros::Publisher cmd_vel_pub_;
	ros::ServiceClient amcl_global_clt_;
	ros::ServiceServer acc_cmd_srv_;
	ros::Publisher move_base_cancel_pub_;
	ros::Subscriber odom_sub_;
	ros::Subscriber bumper_sub_;
	ros::Publisher lift_pub_;
  ros::Publisher lift_publisher; //ADDED BY MUJI
  ros::Subscriber lift_subscriber; // ADDED BY MUJI
	geometry_msgs::Twist man_cmd_vel_;
	MoveBaseClient move_base_clt_; 		/**< Client used to send commands to move_base*/
	std::string frame_id_;
	tf::TransformListener tf_listener_;
	RefuseBin refuse_bin_pose_;
	PidController* velocity_pid_; /**< PID to control position in x*/
	std_msgs::Int8 lift_;

	int state_, loading_state_; 
	int move_base_active_;
	int lifted_; // 1 for lifted 0 for down;
	double current_yaw_;
	double current_vel_;
	bool reversing_;


	double MAX_ANGULAR_VEL;
	double MAX_LINEAR_VEL;
	bool bumperPressed_;
	float target_x_;
	float target_y_;
	float target_yaw_;
	bool holdingBin_; // if a bin is on Chalie, true else false

	//Testing variables here
	double point_goalX;
	double point_goalY;
	//ends here
	
	std::map<std::string, RangeSensor*> range_sensor_list_; /**< List of active rangesensors */
	
	
};

#endif
