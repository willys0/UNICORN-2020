#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

/*ROS*/
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
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

/*Standard Libraries*/
#include <string>
#include <iostream>
#include <termios.h>
#include <cmath>
#include <boost/lexical_cast.hpp>
#include <cstdlib>

/*State classes*/
// #include "unicorn/state.h"
#include "unicorn/idle_state.h"
#include "unicorn/navigating_state.h"
#include "unicorn/aligning_state.h"
#include "unicorn/reversing_state.h"
#include "unicorn/lifting_state.h"
#include "unicorn/state_factory.h"

/*Structure defintions*/
#include "unicorn/state_structures.h"

/*PID controller*/
#include "unicorn/pid_controller.h"

/*User-defined ROS messages*/
#include "unicorn/command.h"

/*Type definitons*/
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; /**< Client that calls actions from move_base */

/*
Description:
The state machine class will handle the overgrasping logic required by the states so they can
transition and responding to events. Furthermore, it also is responsible for keeping the UNICORNs 
*/

class StateMachine
{
public:
    /*Members*/

    /*Methods*/
    /**
     * @brief Default constructor
    */
    StateMachine();
    /**
     * @brief Default de-constructor
    */
    ~StateMachine();
    /**
     * @brief State machine main loop 
    */
    int start();
    
protected:
private:
    /*Members*/
    double max_angular_vel_;
	double max_linear_vel_;
    ros::NodeHandle n_;
    ros::ServiceClient amcl_global_clt_;
    ros::ServiceServer acc_cmd_srv_;
    ros::Publisher state_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber odom_sub_;
	std::string frame_id_;
    const std::string command_topic_ = "commands";
    tf::TransformListener tf_listener_;
	PidController *velocity_pid_; /**< PID to control position in x*/
	RefuseBin refuse_bin_pose_;
    std::shared_ptr<State> current_state_;
    float current_yaw_;
    float current_vel_;
    MoveBaseClient move_base_clt_;

    /*Methods*/
    /**
     * @brief Initialises the global localisation element of the robot with provided parameters specified in the main.launch file.
    */
    void initGlobalLocalisation();
    /**
     * @brief Callback method which captures command messages issued by the command node, or other command interfaces. 
     * 
     * @param msg new command which has been issued.
    */
    void cmdCallback(const unicorn::command &msg);
    /**
     * @brief Callback method which captures and processes messages related to the robot's odometry.
     * 
     * @param msg message containing odometry data.
    */
    void odomCallback(const nav_msgs::Odometry &msg);
    /**
     * @brief Method which publishes a state change to the /TX2_unicorn_state topic. 
     * 
     * @param new_state new state of UNICORN
    */
    void updateAndPublishState(const int new_state);
    /**
     * @brief method which publishes a new goal to the ROS move base
     * 
     * @param x x-coordinate of new goal
     * @param y y-coordinate of new goal
     * @param yaw yaw-coordinate of new goal
    */
    int sendGoal(const float x, const float y, const float yaw);
    /**
     * @brief "fill in brief description"
    */
    bool accGoalServer(unicorn::CharlieCmd::Request &req, unicorn::CharlieCmd::Response &res);
    /**
     * @brief returns the string representation of the current state. 
    */
    std::string getStateString();
};
#endif // !ST