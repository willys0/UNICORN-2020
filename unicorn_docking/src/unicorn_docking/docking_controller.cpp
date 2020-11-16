#include <unicorn_docking/docking_controller.h>

DockingController::DockingController() : nh_("~"), state_(DockingController::DockState::IDLE) {

    apriltag_sub_ = nh_.subscribe("/tag_detections", 100, &DockingController::apriltagDetectionsCb, this);
    state_sub_ = nh_.subscribe("state", 1, &DockingController::stateCb, this);

    // TODO: Load initial gains from parameter server
    pid_x_.initParam("~/pid/x");
    pid_th_.initParam("~/pid/th");

    nh_.param("offset/x",desired_offset_.x, 0.2);
    nh_.param("offset/y",desired_offset_.y, 0.0);
    nh_.param("offset/th",desired_offset_.z, 0.0);
    //pid_x_.initPid(6.0, 1.0, 2.0, 0.3, -0.3, nh_);
    //pid_th_.initPid(6.0, 1.0, 2.0, 0.3, -0.3, nh2_);

    last_time_ = ros::Time::now();

    ROS_INFO("Setting dock offset to x: %.2f, y: %.2f, th: %.2f", desired_offset_.x, desired_offset_.y, desired_offset_.z);
}

double DockingController::getDistanceToTag() {
    // Returns the distance to the tag in the z direction. i.e the distance straight forward in the camera frame.
    return tag_pose_.position.z;
}

double DockingController::getRotationToTag() {
    // Get the pitch of the tag
    double tag_roll, tag_pitch, tag_yaw;
    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(tag_pose_.orientation, quat_tf);
    tf::Matrix3x3(quat_tf).getRPY(tag_roll, tag_pitch, tag_yaw);

    // Get the rotation to the tag
    double position_rot;
    if (tag_pose_.position.x != 0.0) {
        position_rot = asin(tag_pose_.position.z/tag_pose_.position.x);
    }
    else
    {
        position_rot = 0.0;
    }
    


    return position_rot;
}

void DockingController::apriltagDetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    double roll, pitch, yaw;

    for(auto& tag : msg->detections) {
        if(tag.id[0] == 0) {
            // Save tag pose to tag_pose_
            tag_pose_ = tag.pose.pose.pose;

            // Print tag position in z, x and pitch        
            tf2::Quaternion quat_tf;
            tf2::fromMsg(tag_pose_.orientation, quat_tf);
            tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
            ROS_INFO("z: %.2f, x: %.2f, pitch: %.2f", tag_pose_.position.z, tag_pose_.position.x, pitch);
            break;
        }
    }
}

void DockingController::stateCb(const std_msgs::Int32::ConstPtr& msg) {
    setState((DockingController::DockState)msg->data);
}

geometry_msgs::Twist DockingController::computeVelocity() {
    geometry_msgs::Twist msg;
    double err_x, err_th;

    if(state_ == DOCKING) {
        ros::Time current_time = ros::Time::now();

        // Calculate x ans theta errors
        err_x = desired_offset_.x - getDistanceToTag();
        err_th = desired_offset_.z - getRotationToTag();

        // Check if abs errors are outside margins
        if (abs(err_x) > 0.01 || abs(err_th) > 0.02) {
            
            msg.linear.x = pid_x_.computeCommand(desired_offset_.x - getDistanceToTag(), current_time - last_time_);
            // TODO: make actual angle offset for pitch, most likely zero offset, so we dont use z offset.
            msg.angular.z = -pid_th_.computeCommand(desired_offset_.z - getRotationToTag(), current_time - last_time_);

            last_time_ = current_time;
        }
        else {
            // Both errors are within margins, set velocity and rotation to zero 
            ROS_INFO("Docking complete, errors within margin: err_x: %.2f, err_th: %.2f", err_x, err_th);
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
        }

    }
    return msg;
}
