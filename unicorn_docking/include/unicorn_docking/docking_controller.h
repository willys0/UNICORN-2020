#ifndef __DOCKING_CONTROLLER_H_
#define __DOCKING_CONTROLLER_H_

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <control_toolbox/pid.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <std_msgs/Int32.h>

class DockingController {

    public:
        typedef enum DockState { IDLE, DOCKING } DockState;
        DockingController();

        geometry_msgs::Twist computeVelocity();

        double getDistanceToTag();

        double getRotationToTag();
        
        DockState getState() { return state_; }
        void setState(DockState state) { state_ = state; }

    protected:
        void apriltagDetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

        void stateCb(const std_msgs::Int32::ConstPtr& msg);

    private:
        ros::NodeHandle nh_;

        ros::Subscriber apriltag_sub_;
        ros::Subscriber state_sub_;

        control_toolbox::Pid pid_x_;
        control_toolbox::Pid pid_th_;

        geometry_msgs::Pose  tag_pose_;
        geometry_msgs::Point desired_offset_;

        ros::Time last_time_;

        DockState state_;

};


#endif // __DOCKING_CONTROLLER_H_
