#include "unicorn/idle_state.h"

IDLEState::IDLEState()
{
   state_identifier_ = STATE_IDLE;
   command.state = -1;
}

IDLEState::~IDLEState()
{
}

Command IDLEState::run()
{
    ROS_INFO("Waiting for new command...");
    ros::Rate rate(0.3);
    while(command.state == -1)
    {
        rate.sleep();
        ROS_INFO("No new command yet.");
        ros::spinOnce();
    }
    ROS_INFO("New command has been recieved: %d", command.state);
    return command;
}
