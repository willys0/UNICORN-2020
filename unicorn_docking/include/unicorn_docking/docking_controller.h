#ifndef __DOCKING_CONTROLLER_H_
#define __DOCKING_CONTROLLER_H_

#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <control_toolbox/pid.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32.h>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>

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
        DockingController();

        void reset();

        bool computeVelocity(geometry_msgs::Twist& msg_out);

        double getPitchComponent();

        double getLateralComponent();

        double getDistAlongTagNorm();

        double getDesiredRotation();

        double getRotationToTag();

        double fuseDistances(double apriltag_dist, double lidar_dist);

        double fuseAngles(double apriltag_angle, double lidar_angle, double apriltag_dist);

        DockState getState() { return state_; }
        void setState(DockState state) { state_ = state; }

        void setDesiredOffset(geometry_msgs::Point offset) { desired_offset_ = offset; }

        void setDesiredRotationFunctionParameters(double a, double b, double c) {
                rotational_a_ = a;
                rotational_b_ = b;
                rotational_c_ = c;
        }

        void setSpeedLimit(double max_speed) {
            max_docking_speed_ = max_speed;
        }

        void setMaxTfLookupTime(double max_tf_time) {
            max_tf_lookup_time_ = max_tf_time;
        }

        double xError() { return err_x_; }
        double yError() { return err_y_; }
        double thError() { return err_th_; }

    protected:
        void apriltagDetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

        void lidarCb(const sensor_msgs::LaserScanConstPtr& msg);

    private:
        ros::NodeHandle nh_;

        ros::Subscriber apriltag_sub_;
        ros::Subscriber lidar_sub_;

        ros::Publisher  d_pub_;
        ros::Publisher  n_pub_;
        ros::Publisher  desired_rot_pub_;
        ros::Publisher  rot_to_tag_pub_;
        ros::Publisher  tag_pitch_pub_;

        control_toolbox::Pid pid_x_;
        control_toolbox::Pid pid_th_;

        geometry_msgs::Point desired_offset_;
        geometry_msgs::TransformStamped wheelbase_to_tag_tf_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        std::string base_link_frame_;
        std::string lidar_frame_;

        bool use_lidar_;
        double lidar_offset_x_;
        double lidar_dist_;
        double lidar_angle_;

        double err_x_;
        double err_y_;
        double err_th_;

        double rotational_a_;
        double rotational_b_;
        double rotational_c_;

        double max_docking_speed_;
        double max_docking_rotation_speed_;
        double max_tf_lookup_time_;

        ros::Time last_time_;
        ros::Time tag_last_seen_;

        DockState state_;

};


#endif // __DOCKING_CONTROLLER_H_
