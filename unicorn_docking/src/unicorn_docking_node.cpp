#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

geometry_msgs::Pose tagPose;

double currentPosition(void) {
    double dist;
    dist = sqrt(tagPose.position.x*tagPose.position.x+tagPose.position.y*tagPose.position.y);
    // Pos relative to tag
    // WHAT HAPPENS IF TAG NOT VISIBLE?!?!?!?!????? use odom to calculate??
    // WHAT IF OBJECT IN WAY???!?!?!?!????!??????
    return dist;
}

void getPosition(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {

    for(auto& tag : msg->detections) {
        if(tag.id[0] == 0) {
            tagPose = tag.pose.pose.pose;
            break;
        }
    }
}


int main(int argc, char **argv) {
    ros::Publisher vel_pub;
    ros::Subscriber apriltag_sub;

    // Init ros node
    ros::init(argc, argv, "unicorn_docking_node");

    ros::NodeHandle n;

    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    apriltag_sub = n.subscribe("/tag_detections", 100, &getPosition);

    geometry_msgs::Twist move_msg;

    control_toolbox::Pid pid_x;
    pid_x.initPid(6.0, 1.0, 2.0, 0.3, -0.3, n);
    double position_des_ = 0.0;

    ros::Time last_time = ros::Time::now();
    while (ros::ok()) {
        ros::Time time = ros::Time::now();
        double command = pid.computeCommand(position_des_ - currentPosition(), time - last_time);
        last_time = time;

        
        move_msg.linear.x = command;
        vel_pub.publish(move_msg);
        
        ros::spinOnce();
        ros::Duration(1.0/100.0).sleep();
    }
    
    return 0;
}