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

class State
{
    public:
        State();
        ~State();
        virtual int run() = 0;
    protected:
        int abort_state_;
    private:
        ros::Subscriber abort_sub_;
        void AbortCallback(const std_msgs::Int32 &msg);
};