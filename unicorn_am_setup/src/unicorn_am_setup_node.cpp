#include <ros/ros.h>
#include <am_driver/Mode.h>
#include <std_msgs/UInt16.h>


int main(int argc, char **argv) {
    ros::Publisher pub;
    std_msgs::UInt16 msg;

    // Init ros node
    ros::init(argc, argv, "unicorn_am_setup_node");

    // Create a node handle
    ros::NodeHandle n;

    pub = n.advertise<std_msgs::UInt16>("/cmd_mode", 1, true);
    
    while(ros::ok()) {
        msg.data = am_driver::Mode::MODE_LOOP_OFF;
        pub.publish(msg);
        ros::Duration(1).sleep();
    }
    return 0;
}