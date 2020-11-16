#include <ros/ros.h>

#include <geometry_msgs/Twist.h>

#include <unicorn_docking/docking_controller.h>


int main(int argc, char **argv) {
    ros::Publisher vel_pub;

    // Init ros node
    ros::init(argc, argv, "unicorn_docking_node");

    ros::NodeHandle n;

    DockingController docking_controller;
    geometry_msgs::Twist move_msg;

    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    while (ros::ok()) {
        if(docking_controller.getState() == DockingController::DockState::DOCKING) {
            move_msg = docking_controller.computeVelocity();
            vel_pub.publish(move_msg);
        }
        
        ros::spinOnce();
        ros::Duration(1.0/100.0).sleep();
    }
    
    return 0;
}
