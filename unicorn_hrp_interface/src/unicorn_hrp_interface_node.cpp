#include <ros/ros.h>

#include <unicorn_hrp_interface/am_unicorn_interface.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "am_unicorn_interface");
    ROS_INFO("Started automower to unicorn interface");

    AmUnicornInterface am_unicorn_interface;
    ros::Rate r(100.0);

    while(ros::ok())
    {
        ros::spinOnce();

        am_unicorn_interface.publishCmd();
        r.sleep();
    }
}
