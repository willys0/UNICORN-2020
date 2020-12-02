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

        DockState getState() { return state_; }
        void setState(DockState state) { state_ = state; }

        void setDesiredOffset(geometry_msgs::Point offset) { desired_offset_ = offset; }

        void setDesiredRotationFunctionParameters(double a, double b, double c) {
                rotational_a_ = a;
                rotational_b_ = b;
                rotational_c_ = c;
        }

        void setMaxTfLookupTime(double max_tf_time) {
            max_tf_lookup_time_ = max_tf_time;
        }

        void setMinObjectDistances(double min_distance_infront, double min_distance_behind) {
            min_distance_infront_ = min_distance_infront;
            min_distance_behind_ = min_distance_behind;
        }

        double xError() { return err_x_; }
        double yError() { return err_y_; }
        double thError() { return err_th_; }

    protected:
        double getPitchComponent();

        double getLateralComponent();

        double getDistAlongTagNorm();

        double getDesiredRotation();

        bool getRotationToTag(double& rotation_to_tag);

        double getDistanceToClosestObject(const sensor_msgs::LaserScan& laser_scan, double laser_scan_angle);

        double fuseDistances(double apriltag_dist, double lidar_dist, double lidar_angle);

        double fuseAngles(double apriltag_angle, double lidar_angle, double apriltag_dist);

        void apriltagDetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);

        void rearLidarCb(const sensor_msgs::LaserScanConstPtr& msg);

        void frontLidarCb(const sensor_msgs::LaserScanConstPtr& msg);

    private:
        ros::NodeHandle nh_;

        ros::Subscriber apriltag_sub_;
        ros::Subscriber lidar_sub_;
        ros::Subscriber front_lidar_sub_;

        ros::Publisher  d_pub_;
        ros::Publisher  n_pub_;

        control_toolbox::Pid pid_x_;
        control_toolbox::Pid pid_th_;

        geometry_msgs::Point desired_offset_;
        geometry_msgs::TransformStamped wheelbase_to_tag_tf_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        std::string base_link_frame_;
        std::string lidar_frame_;

        bool use_lidar_;
        std::vector<int> lidar_indices_;
        double lidar_offset_x_;
        double lidar_dist_;
        double lidar_angle_;

        double err_x_;
        double err_y_;
        double err_th_;

        double rotational_a_;
        double rotational_b_;
        double rotational_c_;

        double max_tf_lookup_time_ = 5.0;

        double min_distance_infront_;
        double min_distance_behind_;

        double rear_lidar_angle_;
        double front_lidar_angle_;
        sensor_msgs::LaserScan front_lidar_scan_;
        sensor_msgs::LaserScan rear_lidar_scan_;
        double max_time_since_lidar_scan_;

        ros::Time last_time_;
        ros::Time tag_last_seen_;

        DockState state_;

};


#endif // __DOCKING_CONTROLLER_H_
