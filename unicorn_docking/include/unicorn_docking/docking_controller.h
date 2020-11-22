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

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <dynamic_reconfigure/server.h>
#include <unicorn_docking/DockingControllerConfig.h>


class DockingController {

    public:
        typedef enum DockState { IDLE, DOCKING } DockState;
        DockingController();

        void reset();

        bool computeVelocity(geometry_msgs::Twist& msg_out);

        double getDistanceToTag();

        double getPitchComponent();

        double getLateralComponent();

        double getDistAlongTagNorm();

        double getDesiredRotation();

        double getRotationToTag();
        
        DockState getState() { return state_; }
        void setState(DockState state) { state_ = state; }

        void setDesiredOffset(geometry_msgs::Point offset) { desired_offset_ = offset; }

    protected:
        void apriltagDetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

        void dynamicReconfigCallback(unicorn_docking::DockingControllerConfig& config, uint32_t level);

    private:
        ros::NodeHandle nh_;

        ros::Subscriber apriltag_sub_;

        ros::Publisher  detection_pub_;

        dynamic_reconfigure::Server<unicorn_docking::DockingControllerConfig> reconfig_server_;

        control_toolbox::Pid pid_x_;
        control_toolbox::Pid pid_th_;

        geometry_msgs::Pose  tag_pose_;
        geometry_msgs::Point desired_offset_;
        geometry_msgs::Vector3 tag_point_rel_wheel_base_;
        geometry_msgs::Vector3 camera_to_wheelbase_transform;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        std::vector<double> tag_pitch_mean_vec_;
        int nr_for_pitch_average_;   

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