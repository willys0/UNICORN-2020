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

/*State classes*/
#include "idle_state.h"
#include "navigating_state.h"
#include "aligning_state.h"
#include "reversing_state.h"
#include "lifting_state.h"

/*Structure defintions*/
#include "state_structures.h"

/*PID controller*/
#include "pid_controller.h"

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
    StateMachine();
    ~StateMachine();
    int start();
    /*
     * takes the next state and searches the constructor array for the
     * relevant state constructor. Also sends potential move_base
     */
protected:
private:
    /*Members*/
    double current_yaw_;
	double current_vel_;
    const double max_angular_vel_;
	const double max_linear_vel_;
    ros::NodeHandle n_;
    ros::ServiceClient amcl_global_clt_;
    ros::ServiceServer acc_cmd_srv_;
    ros::Publisher state_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber move_base_cancel_pub_;
    ros::Subscriber odom_sub_;
	std::string frame_id_;
    tf::TransformListener tf_listener_;
	PidController *velocity_pid_; /**< PID to control position in x*/
	RefuseBin refuse_bin_pose_;
    MoveBaseClient move_base_clt_;
    State current_state_;
    
    /*Methods*/
    void initGlobalLocalisation();
    void initNextState(struct cmd_struct cmd_struct_);
    void cmdCallback(const std_msgs::String &msg);
    void odomCallback(const nav_msgs::Odometry &msg);
    void updateAndPublishState(const int new_state);
    cmd_struct parseCmdMsg(std::string cmd_msg);
    std::string getStateString();
};
#endif // !ST