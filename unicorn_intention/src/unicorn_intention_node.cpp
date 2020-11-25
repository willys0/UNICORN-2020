#include <ros/ros.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#include <math.h>

#define N 30

void velocityCallback(const geometry_msgs::TwistConstPtr& msg, int* intention_angles, int* intention_avg) {
    *intention_avg = 0;
    for(int i = 0; i < N-1; i++) {
        intention_angles[i+1] = intention_angles[i];
        *intention_avg += intention_angles[i];
    }

    intention_angles[0] = -round((msg->angular.z)*180/M_PI);
    if (intention_angles[0] > 180)
    {
        intention_angles[0] = intention_angles[0] - 360;
    }
    else if (intention_angles[0] < -180)
    {
        intention_angles[0] = intention_angles[0] + 360;
    }

    *intention_avg += intention_angles[0];

    *intention_avg /= N;
    ROS_INFO("Intention angle: %d", *intention_avg);
}

void stateCallback(const std_msgs::Int32ConstPtr& msg, ros::Publisher led_state_pub) {

    static const int unicorn_state_to_led_state[] = { 0, 1, 3, 2, 4 };
    static std_msgs::Int32 led_msg;

    led_msg.data = unicorn_state_to_led_state[msg->data];

    led_state_pub.publish(led_msg);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "unicorn_intention_node");

    ros::NodeHandle nh;

    int intention_angles[N];
    int intention_avg = 0;

    ros::Publisher intention_pub = nh.advertise<std_msgs::Int32>("intention_angle", 1, false);
    ros::Publisher led_state_pub = nh.advertise<std_msgs::Int32>("led_state", 1, true);

    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(velocityCallback, _1, intention_angles, &intention_avg));
    ros::Subscriber state_sub   = nh.subscribe<std_msgs::Int32>("/unicorn/state", 1, boost::bind(stateCallback, _1, led_state_pub));

    std_msgs::Int32 intention_msg;
    while(ros::ok()) {
        ros::spinOnce();
        intention_msg.data = intention_avg;
        intention_pub.publish(intention_msg);

        ros::Duration(1.0/30.0).sleep();
    }

    return 0;
}
