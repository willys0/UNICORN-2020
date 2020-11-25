#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#include <math.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "unicorn_intention_node");

    ros::NodeHandle nh;

    int intention_angle;

    boost::function<void (const geometry_msgs::TwistConstPtr& msg)> cmd_vel_callback =
        [&intention_angle] (const geometry_msgs::TwistConstPtr& msg) {
            intention_angle = -round((msg->angular.z)*180/M_PI);
            if (intention_angle > 180)
            {
                intention_angle = intention_angle - 360;
            }
            else if (intention_angle < -180)
            {
                intention_angle = intention_angle + 360;
            }

            ROS_INFO("Intention angle: %d", intention_angle);
        };

    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, cmd_vel_callback);

    ros::Publisher intention_pub = nh.advertise<std_msgs::Int32>("intention_angle", 1, false);

    std_msgs::Int32 intention_msg;
    while(ros::ok()) {
        ros::spinOnce();
        intention_msg.data = intention_angle;
        intention_pub.publish(intention_msg);

        ros::Duration(1.0/30.0).sleep();
    }

    return 0;
}
