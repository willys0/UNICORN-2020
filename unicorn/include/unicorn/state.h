#ifndef STATE_H
#define STATE_H

/**
 * Base class for states in UNICORN
 */
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
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

/*Standard Libraries*/
#include <string>
#include <iostream>
#include <termios.h>
#include <cmath>
#include <boost/lexical_cast.hpp>
#include <cstdlib>
/* JSON */

/* UNICORN */
#include "unicorn/state_structures.h"

// for convenience

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; /**< Client that calls actions from move_base */

class State
{
public:
    State(){};
    State(ros::NodeHandle node);
    ~State(){};
    Command setNewCmd(Command new_cmd) { ROS_INFO("New command has been set"); command = new_cmd; };
    virtual Command run() = 0;
    int state_identifier_;
protected:
    std::string cmd_msg_str_;
    Command command;
    
private:
};
#endif // !ST