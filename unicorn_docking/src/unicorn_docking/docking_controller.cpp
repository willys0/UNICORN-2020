#include <unicorn_docking/docking_controller.h>

DockingController::DockingController() : nh_("~"), state_(DockingController::DockState::IDLE) {

    apriltag_sub_ = nh_.subscribe("/tag_detections", 100, &DockingController::apriltagDetectionsCb, this);

    // TODO: Load initial gains from parameter server
    pid_x_.initParam("~/pid/x");
    pid_control_th_.initParam("~/pid/control_th");
    pid_th_.initParam("~/pid/th");

    // load settings
    nh_.param("max_retries", max_retries_, 3);
    nh_.param("retry_offset", retry_offset_, 0.6);
    nh_.param("thresh_x", thresh_x_, 0.01);
    nh_.param("thresh_y", thresh_y_, 0.01);
    nh_.param("thresh_th", thresh_th_, 0.01);
    nh_.param("retry_error_times", retry_error_times_, 50);


    //pid_x_.initPid(6.0, 1.0, 2.0, 0.3, -0.3, nh_);
    //pid_th_.initPid(6.0, 1.0, 2.0, 0.3, -0.3, nh2_);

    retrying_ = false;
    nr_retries_ = 0;
    error_times_ = 0;
    last_time_ = ros::Time::now();

}

double DockingController::getDistanceToTag() {
    // Returns the distance to the tag in the z direction. i.e the distance straight forward in the camera frame.
    return tag_pose_.position.z;
}

double DockingController::getPitchComponent() {
    // Get the pitch of the tag
    double tag_roll, tag_pitch, tag_yaw;
    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(tag_pose_.orientation, quat_tf);
    tf::Matrix3x3(quat_tf).getRPY(tag_roll, tag_pitch, tag_yaw);

    return tag_pitch;
}

double DockingController::getLateralComponent() {

    // get the rotation to the perpendicular of the tag normal. ?
    double a, d, pitch, pi_4, amplitude, rot_to_tag,dist_to_tag;
    //pi_4 = 2*3.14159265359/3;

    pitch = getPitchComponent();
    dist_to_tag = sqrt(tag_pose_.position.z*tag_pose_.position.z + tag_pose_.position.x*tag_pose_.position.x);
    d = dist_to_tag*sin(pitch);
    
    // robot is on left side of normal from apriltag
    if (pitch > 0) {
        return d;
    }
    else
    {
        return d;
    }
    
    // a = tag_pose_.position.z*cos(pitch);
    // WHAT TO DO???????
    // amplitude = d/a;

    // // rotation from camera to tag
    // if (tag_pose_.position.z == 0.0) {
    //     rot_to_tag = 0.0;
    // }
    // else {
    //     rot_to_tag = asin(tag_pose_.position.x/tag_pose_.position.z);
    // }
    
    // // robot is on left side of normal from apriltag
    // if (pitch > 0) {
    //     // rotation 
    //     if (rot_to_tag < -pi_4) {
    //         return -amplitude*pitch;
    //     }
    //     else {
    //         return amplitude*pitch;
    //     }
    // }
    // else {
    //     // rotation 
    //     if (rot_to_tag > pi_4) {
    //         return amplitude*pitch;
    //     }
    //     else {
    //         return -amplitude*pitch;
    //     }
    // }
    
}

double DockingController::getRotationToTag() {
    double rot_to_tag;
    // rotation from camera to tag
    if (tag_pose_.position.z == 0.0) {
        rot_to_tag = 0.0;
    }
    else {
        rot_to_tag = asin(tag_pose_.position.x/tag_pose_.position.z);
    }
    return rot_to_tag;
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
            //ROS_INFO("z: %.2f, x: %.2f, pitch: %.2f, d: %.2f", tag_pose_.position.z, tag_pose_.position.x, pitch, getLateralComponent());
            break;
        }
    }
}

bool DockingController::computeVelocity(geometry_msgs::Twist& msg_out) {
    double err_x, err_y, err_th, des_rot;

    if(state_ == DOCKING) {
        ros::Time current_time = ros::Time::now();

        // Calculate x ans theta errors
        err_x = desired_offset_.x - getDistanceToTag();
        err_y = desired_offset_.y - tag_pose_.position.y;
        err_th = desired_offset_.z - getRotationToTag();


        if(retrying_ == true){
            err_x = retry_offset_ - getDistanceToTag();
            if(fabs(err_x) <= thresh_x_){
                retrying_ = false;
            }
            else{
                msg_out.linear.x = pid_x_.computeCommand(retry_offset_ - getDistanceToTag(), current_time - last_time_);
                // Maby okay with negative
                //des_rot = -pid_control_th_.computeCommand(desired_offset_.z - getLateralComponent(), current_time - last_time_);
                msg_out.angular.z = -pid_th_.computeCommand(desired_offset_.z - getPitchComponent(), current_time - last_time_);

                last_time_ = current_time;
            }
            ROS_INFO("RETRY #: %d, err_x: %.2f, err_y: %.2f, err_th: %.2f", nr_retries_, err_x, err_y, err_th);
            
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
                    else
                    {
                        error_times_++;
                    }
                    
                    
                    
                }
                else {
                    // Move towards desired position
                    msg_out.linear.x = pid_x_.computeCommand(desired_offset_.x - getDistanceToTag(), current_time - last_time_);
                    des_rot = pid_control_th_.computeCommand(desired_offset_.y - getLateralComponent(), current_time - last_time_);
                    msg_out.angular.z = pid_th_.computeCommand(des_rot - getRotationToTag(), current_time - last_time_);

                    last_time_ = current_time;
                    ROS_INFO("des_rot: %.2f", des_rot);
                }
                

            }
            else {
                if(fabs(err_x) <= thresh_x_) {
                    // All errors are within margins, set velocity and rotation to zero 
                    ROS_INFO("Docking complete, errors within margin: err_x: %.2f, err_y: %.2f, err_th: %.2f", err_x, err_y, err_th);
                    msg_out.linear.x = 0.0;
                    msg_out.angular.z = 0.0;
                    return true;
                }
            }
            
        }
        return false;
    }

    return false;
}
