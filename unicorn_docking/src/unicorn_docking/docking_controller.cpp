#include <unicorn_docking/docking_controller.h>

DockingController::DockingController() : nh_("~"), state_(DockingController::DockState::IDLE), tf_listener_(tf_buffer_) {

    apriltag_sub_ = nh_.subscribe("/tag_detections", 100, &DockingController::apriltagDetectionsCb, this);
    d_pub_ = nh_.advertise<visualization_msgs::Marker>("d_tag", 1);
    n_pub_ = nh_.advertise<visualization_msgs::Marker>("n_tag", 1);
    desired_rot_pub_ = nh_.advertise<std_msgs::Float64>("desired_rot", 1);
    rot_to_tag_pub_ = nh_.advertise<std_msgs::Float64>("rot_to_tag", 1);
    tag_pitch_pub_ = nh_.advertise<std_msgs::Float64>("tag_pitch", 1);

    // TODO: Load initial gains from parameter server
    pid_x_.initParam("~/pid/x");
    pid_th_.initParam("~/pid/th");

    nh_.param("base_link_frame", base_link_frame_, std::string("chassis_link"));

    reset();

}

void DockingController::reset() {
    last_time_ = ros::Time::now();

    tag_last_seen_ = ros::Time::now();
}


double DockingController::getPitchComponent() {
    double tag_roll, tag_pitch, tag_yaw;

    // Gets the pitch of the tag
    tf2::getEulerYPR(wheelbase_to_tag_tf_.transform.rotation, tag_yaw, tag_pitch, tag_roll);

    // Account for rotation in transform from chassis_link to DOCK_BUNDLE frame 
    tag_pitch += 1.57079632679;
    
    // Set the correct sign of the pitch
    if(tag_roll < 0){
        tag_pitch = -tag_pitch;
    }

    return tag_pitch;
}

double DockingController::getLateralComponent() {
    // Gets the distance perpendicular from the tag normal to the robot. (along tags x axis)
    
    return wheelbase_to_tag_tf_.transform.translation.x;
}

double DockingController::getDistAlongTagNorm() {
    // Gets the distance to the robot along the tag normal. (along tags z axis)
    return wheelbase_to_tag_tf_.transform.translation.z;
}

double DockingController::getDesiredRotation() {
    double d, des_rot;

    d = getLateralComponent();
    
    // Calculate desired rotation based on lateral displacement
    des_rot = rotational_a_ + exp(rotational_b_ * fabs(d) + rotational_c_);

    // Cap totation at max pi/2
    if(des_rot > 0.4){
        des_rot = 0.4;
    }

    // give rotation correct sign
    if(d > 0){
        des_rot = -1*des_rot;
    }

    return des_rot;
}

double DockingController::getRotationToTag() {
    double rot_to_tag;
    geometry_msgs::TransformStamped dock_chassi_tf;
    // rotation from base_link_frame to tag normal (tag z axis)

    // Try to get transform from DOCK_BUNDLE to base_link_frame_
    try {
        dock_chassi_tf = tf_buffer_.lookupTransform(base_link_frame_, ros::Time::now(), "DOCK_BUNDLE", tag_last_seen_, "map", ros::Duration(max_tf_lookup_time_));
    }
    catch(tf2::TransformException &ex) {
        ROS_WARN("in getRotationToTag(): %s",ex.what());
        return -100;
    }
    // Rotation from chassis_ling frame to DOCK_BUNDLE frame
    rot_to_tag = atan2(dock_chassi_tf.transform.translation.y, -dock_chassi_tf.transform.translation.x);

    return rot_to_tag;
}


void DockingController::apriltagDetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    static int seq = 0;
    for(auto& tag : msg->detections) {
        if(tag.id[0] == 0 && tag.id[1] == 4 && tag.id[2] == 3 && tag.id[3] == 2 && tag.id[4] == 1) {
            
            // =============================================================
            // Visualisation markers for debug
            // =============================================================
            visualization_msgs::Marker d_pose_msg, n_pose_msg;
            geometry_msgs::Quaternion quat_msg;
            quat_msg = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
            d_pose_msg.header.frame_id = "DOCK_BUNDLE";
            d_pose_msg.header.stamp = ros::Time::now();
            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            d_pose_msg.ns = "n_distance";
            d_pose_msg.id = 0;
            // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
            d_pose_msg.type = visualization_msgs::Marker::ARROW;
            // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
            d_pose_msg.action = visualization_msgs::Marker::ADD;
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            d_pose_msg.pose.position.x = 0.0;
            d_pose_msg.pose.position.y = 0.0;
            d_pose_msg.pose.position.z = wheelbase_to_tag_tf_.transform.translation.z;
            d_pose_msg.pose.orientation = quat_msg;
            
            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            d_pose_msg.scale.x = wheelbase_to_tag_tf_.transform.translation.x;
            d_pose_msg.scale.y = 0.1;
            d_pose_msg.scale.z = 0.1;
            // Set the color -- be sure to set alpha to something non-zero!
            d_pose_msg.color.r = 0.0f;
            d_pose_msg.color.g = 1.0f;
            d_pose_msg.color.b = 0.0f;
            d_pose_msg.color.a = 1.0;
            d_pose_msg.lifetime = ros::Duration();

            d_pub_.publish(d_pose_msg);

            quat_msg = tf::createQuaternionMsgFromRollPitchYaw(0.0, -1.57079632679, 0.0);
            n_pose_msg.header.frame_id = "DOCK_BUNDLE";
            n_pose_msg.header.stamp = ros::Time::now();
            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            n_pose_msg.ns = "n_distance";
            n_pose_msg.id = 0;
            // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
            n_pose_msg.type = visualization_msgs::Marker::ARROW;
            // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
            n_pose_msg.action = visualization_msgs::Marker::ADD;
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            n_pose_msg.pose.position.x = 0.0;
            n_pose_msg.pose.position.y = 0.0;
            n_pose_msg.pose.position.z = 0.0;
            n_pose_msg.pose.orientation = quat_msg;

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            n_pose_msg.scale.x = wheelbase_to_tag_tf_.transform.translation.z;
            n_pose_msg.scale.y = 0.1;
            n_pose_msg.scale.z = 0.1;
            // Set the color -- be sure to set alpha to something non-zero!
            n_pose_msg.color.r = 0.0f;
            n_pose_msg.color.g = 1.0f;
            n_pose_msg.color.b = 0.0f;
            n_pose_msg.color.a = 1.0;
            n_pose_msg.lifetime = ros::Duration();

            n_pub_.publish(n_pose_msg);
            //======================================================================================

            //======================================================================================
            // Publish desired rotation, rotation to tag and tag pitch
            //======================================================================================
            std_msgs::Float64 f;
            f.data = getDesiredRotation();
            desired_rot_pub_.publish(f);
            
            f.data = getRotationToTag();
            rot_to_tag_pub_.publish(f);

            f.data = getPitchComponent();
            tag_pitch_pub_.publish(f);
            //======================================================================================

            // Set time at witch the tag was last seen
            tag_last_seen_ = tag.pose.header.stamp;

            break;
        }
    }
}

bool DockingController::computeVelocity(geometry_msgs::Twist& msg_out) {

    ros::Time current_time = ros::Time::now();

    
    // Try to timetravel!
    // Try to find transform between DOCK_BUNDLE frame at tag_last_seen_ time and chassis_link at current_time
    try {
        wheelbase_to_tag_tf_ = tf_buffer_.lookupTransform("DOCK_BUNDLE", tag_last_seen_, base_link_frame_, current_time, "map", ros::Duration(max_tf_lookup_time_));
    }
    catch(tf2::TransformException &ex) {
        // If no transform could be found, set linear and angular velocities to zero
        ROS_WARN("in computeVelocity(): %s",ex.what());
        msg_out.linear.x = 0.0;
        msg_out.angular.z = 0.0;
        return false;
    }
    

    if(state_ == DOCKING) {

        // Calculate x, y and theta errors
        err_x_ = desired_offset_.x - getDistAlongTagNorm();
        err_y_ = desired_offset_.y - getLateralComponent();
        err_th_ = desired_offset_.z - getPitchComponent();

        msg_out.linear.x = pid_x_.computeCommand(desired_offset_.x - getDistAlongTagNorm(), current_time - last_time_);

        // If linear.x is positive the rotation needs to be the other way as it is moving away from the tag and not towards it.
        if(msg_out.linear.x >= 0) {
            msg_out.angular.z = pid_th_.computeCommand(-getDesiredRotation() - getRotationToTag(), current_time - last_time_);
        }
        else {
            msg_out.angular.z = pid_th_.computeCommand(getDesiredRotation() - getRotationToTag(), current_time - last_time_);
        }
        

        if(msg_out.linear.x > max_docking_speed_) {
            msg_out.linear.x = max_docking_speed_;
        }
        else if(msg_out.linear.x < -max_docking_speed_) {
            msg_out.linear.x = -max_docking_speed_;
        }


        last_time_ = current_time;
    }

    return false;
}
