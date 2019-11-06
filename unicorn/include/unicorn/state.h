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
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

/* C / C++ */
#include <iostream>
#include <termios.h>
#include <cmath>
#include <boost/lexical_cast.hpp>
#include <string>

/* JSON */
#include <nlohmann/json.hpp>

/* UNICORN */
#include "state_structures.h"

// for convenience
using json = nlohmann::json;

class State
{
public:
    State(){};
    ~State(){};
    Command setNewCmd(Command new_cmd) { command = new_cmd; };
    virtual Command run() = 0;

protected:
    int abort_state_;
    std::string cmd_msg_str_;
    Command command;

private:
};
#endif // !ST