#include <ros/ros.h>

#include <algorithm>

#include <actionlib/server/simple_action_server.h>

#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>

#include <unicorn_roborio_bridge/RioMasterMsg.h>
#include <unicorn_roborio_bridge/RunLiftAction.h>

#include <unicorn_roborio_bridge/lift_interface.h>
#include <unicorn_roborio_bridge/r2100_interface.h>
#include <unicorn_roborio_bridge/uwb_interface.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define RIO_LIFT_ACTION_STOP    0
#define RIO_LIFT_ACTION_PICKUP  1
#define RIO_LIFT_ACTION_DROPOFF 2

typedef actionlib::SimpleActionServer<unicorn_roborio_bridge::RunLiftAction> LiftActionServer;

ros::Subscriber rio_sub;

LiftInterface* lift_interface;
UwbInterface*  uwb_interface;
R2100Interface* r2100_interface;

void execute_lift(const unicorn_roborio_bridge::RunLiftGoalConstPtr& goal, LiftActionServer* as, ros::NodeHandle nh) {
    std_msgs::Int32 msg;
    int dir;

    if(goal->direction == goal->DIRECTION_PICKUP) {
        ROS_INFO("[Roborio Bridge] Running lift pickup routine.");
        lift_interface->startPickupRoutine();
    }
    else if(goal->direction == goal->DIRECTION_DROPOFF) {
        ROS_INFO("[Roborio Bridge] Running lift dropoff routine.");
        lift_interface->startDropoffRoutine();
    }
    else {
        // TODO: Handle wrong input value
        ROS_ERROR("[Roborio Bridge] Invalid lift direction given to lifting action.");
        as->setAborted();
        return;
    }

    // Wait for lift to move out of idle state
    while(ros::ok() && lift_interface->isIdle()) {
        if(as->isPreemptRequested()) {
            break;
        }

        // TODO: May need to retry sending message here

        ros::spinOnce();
    }

    ROS_INFO("[Roborio Bridge] Waiting for lift to finish...");

    // Wait for lift to be done
    while(ros::ok()) {
        if(as->isPreemptRequested()) {

            ROS_INFO("[Roborio Bridge] Lift preemption requested, stopping and resetting lift...");

            lift_interface->cancelLift();

            ROS_INFO("[Roborio Bridge] Lift action preempted.");
            as->setPreempted();

            // TODO: Shall we reverse the lift back to initial position?
            break;
        }

        if(lift_interface->isIdle()) {
            ROS_INFO("[Roborio Bridge] Lift action succeeded.");
            as->setSucceeded();
            break;
        }
        else if(lift_interface->isErrored()) {
            // TODO: Handle error better
            ROS_ERROR("[Roborio Bridge] Lift action failed.");
            as->setAborted();
            break;
        }

        ros::spinOnce();
    }

}

void rio_cb(const unicorn_roborio_bridge::RioMasterMsgConstPtr& msg) {

    lift_interface->setRioLiftState(msg->lift_state);

    // TODO: Not very optimized, lazy solution. Fix later
    r2100_interface->setLidarScanRanges(std::vector<float>(msg->lidar_ranges.begin(), msg->lidar_ranges.end()));

    uwb_interface->setUwbPosition(msg->uwb_pos);
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "unicorn_roborio_bridge_node");

    ros::NodeHandle nh;

    lift_interface  = new LiftInterface(nh);
    uwb_interface   = new UwbInterface(nh);
    r2100_interface = new R2100Interface(nh);

    rio_sub = nh.subscribe("/RIO_publisher", 10, &rio_cb);

    LiftActionServer lift_server(nh, "/lift/lift_action", boost::bind(&execute_lift, _1, &lift_server, nh), false);
    lift_server.start();


    ros::spin();
}
