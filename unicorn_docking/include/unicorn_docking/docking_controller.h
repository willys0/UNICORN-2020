#ifndef __DOCKING_CONTROLLER_H_
#define __DOCKING_CONTROLLER_H_

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <control_toolbox/pid.h>

#include <apriltag_ros/AprilTagDetectionArray.h>

class DockingController {

    public:
        DockingController();

        geometry_msgs::Twist computeVelocity();

        double getDistanceToTag();

    protected:
        void apriltagDetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

    private:
        ros::NodeHandle nh_;

        ros::Subscriber apriltag_sub_;

        control_toolbox::Pid pid_x_;
        control_toolbox::Pid pid_th_;

        geometry_msgs::Pose  tag_pose_;
        geometry_msgs::Point desired_offset_;

        ros::Time last_time_;
};


#endif // __DOCKING_CONTROLLER_H_