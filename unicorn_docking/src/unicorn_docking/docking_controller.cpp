#include <unicorn_docking/docking_controller.h>

#include <geometry_msgs/PoseStamped.h>

DockingController::DockingController() : nh_("~"), state_(DockingController::DockState::IDLE), tf_listener_(tf_buffer_) {

    apriltag_sub_ = nh_.subscribe("/tag_detections", 100, &DockingController::apriltagDetectionsCb, this);
    detection_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("detected_tag", 1);
    // TODO: Load initial gains from parameter server
    pid_x_.initParam("~/pid/x");
    pid_th_.initParam("~/pid/th");

    // load settings
    nh_.param("max_retries", max_retries_, 3);
    nh_.param("retry_offset", retry_offset_, 0.6);
    nh_.param("thresh_x", thresh_x_, 0.01);
    nh_.param("thresh_y", thresh_y_, 0.01);
    nh_.param("thresh_th", thresh_th_, 0.01);
    nh_.param("retry_error_times", retry_error_times_, 50);
    nh_.param("nr_for_pitch_average",nr_for_pitch_average_, 15);

    tag_pitch_mean_vec_.resize(nr_for_pitch_average_, 0.0);

    // Get transform from camera frame to chassi frame
    geometry_msgs::TransformStamped wheel_base_transform;
    try
    {
        wheel_base_transform = tf_buffer_.lookupTransform("chassis_link","realsense_camera",ros::Time(0),ros::Duration(1));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        //return 0;
    }

    // For wheel_base_transform.transform.translation:
    // (the distance is from realsense_camera to chassis_link frame.)
    // z is (0.34)
    // x is (-0.41)
    // y is (-0.04)

    // Get left/right distance relative to wheel base
    camera_to_wheelbase_transform.x = wheel_base_transform.transform.translation.y;

    // Height of tag isn't used, no need to transform
    camera_to_wheelbase_transform.y = 0;

    // Get the tag distance in relation to wheel base
    camera_to_wheelbase_transform.z = -wheel_base_transform.transform.translation.x;

    last_time_ = ros::Time::now();

}

double DockingController::getDistanceToTag() {
    // Returns the distance to the tag in the z direction. i.e the distance straight forward in the camera frame.
    return tag_pose_.position.z;
}

double DockingController::getPitchComponent() {
    // Get the pitch of the tag
    double tag_roll, tag_pitch, tag_yaw, tag_pitch_med;
    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(tag_pose_.orientation, quat_tf);
    tf::Matrix3x3(quat_tf).getRPY(tag_roll, tag_pitch, tag_yaw);


    for(int i = 1; i < nr_for_pitch_average_ - 1; i++) {
        tag_pitch_mean_vec_[i+1] = tag_pitch_mean_vec_[i];
        tag_pitch_med += tag_pitch_mean_vec_[i];
    }

    tag_pitch_mean_vec_[0] = tag_pitch;
    tag_pitch_med += tag_pitch;

    tag_pitch_med = tag_pitch_med/tag_pitch_mean_vec_.size();


    return tag_pitch_med;
}

double DockingController::getLateralComponent() {
    // Gets the distance perpendicular from the tag normal to the robot.
    
    double d, pitch, dist_to_tag;

    pitch = getPitchComponent();

    dist_to_tag = sqrt(tag_point_rel_wheel_base_.z*tag_point_rel_wheel_base_.z + tag_point_rel_wheel_base_.x*tag_point_rel_wheel_base_.x);
    d = dist_to_tag*sin(getRotationToTag()+pitch);
    
    return d;
    
}

double DockingController::getDistAlongTagNorm() {
    double d, pitch, dist_to_tag;

    pitch = getPitchComponent();

    dist_to_tag = sqrt(tag_pose_.position.z*tag_pose_.position.z + tag_pose_.position.x*tag_pose_.position.x);
    d = dist_to_tag*cos(getRotationToTag()+pitch);
    
    return d;
}

double DockingController::getDesiredRotation() {
    double d, des_rot;
    d = getLateralComponent();
    
    des_rot = exp(6*abs(d)-3);

    // Cap totation at max pi/2
    if(des_rot > 1.5){
        des_rot = 1.5;
    }
    // Rotation from image norm to tag norm
    if(d > 0){
        des_rot = -1*des_rot;
    }

    return des_rot;
}



double DockingController::getRotationToTag() {
    double rot_to_tag;
    // rotation from camera to tag
    if (tag_point_rel_wheel_base_.z == 0.0) {
        rot_to_tag = 0.0;
    }
    else {
        rot_to_tag = atan2(tag_point_rel_wheel_base_.x,tag_point_rel_wheel_base_.z);
    }
    return rot_to_tag;
}


void DockingController::apriltagDetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    static int seq = 0;
    for(auto& tag : msg->detections) {
        if(tag.id[0] == 0) {
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.pose = tag.pose.pose.pose;
            pose_msg.header.frame_id = tag.pose.header.frame_id;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.seq = seq++;

            detection_pub_.publish(pose_msg);

            // Save tag pose to tag_pose_
            // For tag_pose_.position:
            // z is depth of tag in camera frame. Higher value equals further away.
            // x is position of tag left and right of screen center. Right side is positive, left is degative.
            // y is position of tag  upp and down.
            tag_pose_ = tag.pose.pose.pose;

            tag_point_rel_wheel_base_.x = tag_pose_.position.x + camera_to_wheelbase_transform.x;
            tag_point_rel_wheel_base_.y = tag_pose_.position.y + camera_to_wheelbase_transform.y;
            tag_point_rel_wheel_base_.z = tag_pose_.position.z + camera_to_wheelbase_transform.z;

            ROS_INFO("pitch: %+.2f, rot_to_tag: %+.2f, z: %+.2f, x: %+.2f d: %.2f, nz: %.2f", 
            getPitchComponent(), getRotationToTag(), tag_pose_.position.z, tag_pose_.position.x, getLateralComponent(), getDistAlongTagNorm());

            break;
        }
    }
}

bool DockingController::computeVelocity(geometry_msgs::Twist& msg_out) {
    double err_x, err_y, err_th, des_rot;

    if(state_ == DOCKING) {
        ros::Time current_time = ros::Time::now();

        // Calculate x, y and theta errors
        err_x = desired_offset_.x - getDistAlongTagNorm();
        err_y = desired_offset_.y - getLateralComponent();
        err_th = desired_offset_.z - getPitchComponent();


        if(retrying_ == true){
            // Calculate x errors
            err_x = retry_offset_ - getDistanceToTag();
            if(fabs(err_x) <= 4*thresh_x_){
                retrying_ = false;
            }
            else{
                // TODO: MAKE BACKING UPP BETTER
                // ################################################################
                msg_out.linear.x = pid_x_.computeCommand(retry_offset_ - getDistanceToTag(), current_time - last_time_);
                msg_out.angular.z = pid_th_.computeCommand(getDesiredRotation() - getRotationToTag(), current_time - last_time_);

                last_time_ = current_time;
            }
            ROS_INFO("RETRY #: %d, err_x: %.2f, err_y: %.2f, err_th: %.2f, des_rot: %.2f", nr_retries_, err_x, err_y, err_th, getDesiredRotation());
            
        }
        else{
            // If y error or theta error is above their respective thresholds
            if(fabs(err_y) > thresh_y_ || fabs(err_th) > thresh_th_) {
                // If x position is below error threshold
                if(fabs(err_x) <= thresh_x_) {
                    if (error_times_ > retry_error_times_) {
                        error_times_ = 0;
                        // If max number of retries have not been reached
                        if(nr_retries_ < max_retries_) {
                            // Enter retrying_ state
                            retrying_ = true;
                            nr_retries_++;
                        }
                        else {
                            // Max numbers of retries reached without successful docking, Docking FAILED
                            // TODO: DOCKING failed command!
                            // ##########################################################################################################################
                        }

                    }
                    else {
                        error_times_++;
                    }
                }
                else {
                    // Move towards desired position
                    msg_out.linear.x = pid_x_.computeCommand(desired_offset_.x - getDistanceToTag(), current_time - last_time_);
                    msg_out.angular.z = pid_th_.computeCommand(getDesiredRotation() - getRotationToTag(), current_time - last_time_);

                    last_time_ = current_time;
                    //ROS_INFO("err_x: %.2f, err_y: %.2f, err_th: %.2f, des_rot: %.2f, norm_x: %.2f", err_x, err_y, err_th, getDesiredRotation(), getDistAlongTagNorm());
                }
            }
            else {
                if(fabs(err_x) <= thresh_x_) {
                    // All errors are within margins, set velocity and rotation to zero 
                    ROS_INFO("Docking complete after %d retries, errors within margin: err_x: %.2f, err_y: %.2f, err_th: %.2f", nr_retries_, err_x, err_y, err_th);
                    msg_out.linear.x = 0.0;
                    msg_out.angular.z = 0.0;
                    return true;
                }
            }
            
        }
    }

    return false;
}
