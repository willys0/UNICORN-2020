#ifndef __GOAL_H_
#define __GOAL_H_

#include <geometry_msgs/Pose.h>

enum LiftCommand { None = 0, Pickup = 1, Dropoff = 2};

struct Goal {
    geometry_msgs::Pose pose;
    LiftCommand lift_cmd;

};


#endif // __GOAL_H_
