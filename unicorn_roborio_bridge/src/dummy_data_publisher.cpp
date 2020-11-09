/*
** dummy_data_publisher.cpp
**
** Publishes mock rear LiDAR data to test the roboRIO bridge
 */

#include <ros/ros.h>

#include <stdlib.h>

#include <unicorn_roborio_bridge/RioMasterMsg.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "dummy_rio_data_publisher");

    ros::NodeHandle nh;

    unicorn_roborio_bridge::RioMasterMsg msg;

    ros::Publisher pub = nh.advertise<unicorn_roborio_bridge::RioMasterMsg>("/RIO_publisher", 10);

    msg.lidar_ranges.resize(11);
    while(ros::ok()) {

        for(int i = 0; i < 11; i++) {
            float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX) + 0.1;

            msg.lidar_ranges[i] = r;
        }

        pub.publish(msg);
        ros::spinOnce();
    }

}
