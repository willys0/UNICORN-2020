#ifndef __DOCKING_CONTROLLER_H_
#define __DOCKING_CONTROLLER_H_

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <control_toolbox/pid.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <std_msgs/Int32.h>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>


class DockingController {

    public:
        typedef enum DockState { IDLE, DOCKING } DockState;
        DockingController();

        bool computeVelocity(geometry_msgs::Twist& msg_out);

        double getDistanceToTag();

        double getPitchComponent();

        double getLateralComponent();

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
        control_toolbox::Pid pid_control_th_;
        control_toolbox::Pid pid_th_;

        geometry_msgs::Pose  tag_pose_;
        geometry_msgs::Point desired_offset_;
        
        bool retrying_;
        int max_retries_;
        int nr_retries_;
        double retry_offset_;
        int error_times_;
        
        int retry_error_times_;
        double thresh_x_;
        double thresh_y_;
        double thresh_th_;

        ros::Time last_time_;

        DockState state_;

};


#endif // __DOCKING_CONTROLLER_H_
