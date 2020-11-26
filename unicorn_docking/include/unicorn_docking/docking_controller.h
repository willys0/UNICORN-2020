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

// #include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Float64.h>


class DockingController {

    public:
        typedef enum DockState { IDLE, DOCKING } DockState;
        DockingController(int nr_for_pitch_average);

        void reset();

        bool computeVelocity(geometry_msgs::Twist& msg_out);

        // double getDistanceToTag();

        double getPitchComponent();

        double getLateralComponent();

        double getDistAlongTagNorm();

        double getDesiredRotation();

        double getRotationToTag();

        DockState getState() { return state_; }
        void setState(DockState state) { state_ = state; }

        void setDesiredOffset(geometry_msgs::Point offset) { desired_offset_ = offset; }

        void setDesiredRotationFunctionParameters(double a, double b, double c) {
                rotational_a_ = a;
                rotational_b_ = b;
                rotational_c_ = c;
        }

        double xError() { return err_x_; }
        double yError() { return err_y_; }
        double thError() { return err_th_; }

    protected:
        void apriltagDetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

    private:
        ros::NodeHandle nh_;

        ros::Subscriber apriltag_sub_;

        ros::Publisher  detection_pub_;

        ros::Publisher  d_pub_;
        ros::Publisher  n_pub_;
        ros::Publisher  desired_rot_pub_;
        ros::Publisher  rot_to_tag_pub_;
        ros::Publisher  tag_pitch_pub_;

        control_toolbox::Pid pid_x_;
        control_toolbox::Pid pid_th_;

        geometry_msgs::Pose  tag_pose_;
        geometry_msgs::Point desired_offset_;
        geometry_msgs::Vector3 tag_point_rel_wheel_base_;
        geometry_msgs::Vector3 camera_to_wheelbase_transform_;
        geometry_msgs::TransformStamped wheelbase_to_tag_tf_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        bool tag_visible_;
        int error_times_;
        
        int retry_error_times_;

        double err_x_;
        double err_y_;
        double err_th_;

        double rotational_a_;
        double rotational_b_;
        double rotational_c_;

        ros::Time last_time_;

        DockState state_;

};


#endif // __DOCKING_CONTROLLER_H_
