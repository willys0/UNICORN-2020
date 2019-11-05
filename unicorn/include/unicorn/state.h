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

/* C / C++ */
#include <iostream>
#include <termios.h>
#include <cmath>
#include <boost/lexical_cast.hpp>
#include <string>

/*JSON*/
#include <nlohmann/json.hpp>

/*UNICORN*/
#include "state_machine.h"

// for convenience
using json = nlohmann::json;

class State
{
    public:
        State();
        ~State();
        virtual cmd_struct_ run() = 0;
        cmd_msg_struct parseMsg(std::string &msg);

    protected:
        int abort_state_;
        int state_identifier_;
        std::string cmd_msg_str_;        
        struct cmd_msg_struct cmd_struct_;
    private:

};