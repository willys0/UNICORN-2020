#include <unicorn_docking/docking_controller.h>

DockingController::DockingController(int nr_for_pitch_average) : nh_("~"), nr_for_pitch_average_(nr_for_pitch_average), state_(DockingController::DockState::IDLE), tf_listener_(tf_buffer_) {

    apriltag_sub_ = nh_.subscribe("/tag_detections", 100, &DockingController::apriltagDetectionsCb, this);
    detection_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("detected_tag", 1);
    d_pub_ = nh_.advertise<visualization_msgs::Marker>("d_tag", 1);
    n_pub_ = nh_.advertise<visualization_msgs::Marker>("n_tag", 1);
    desired_rot_pub_ = nh_.advertise<std_msgs::Float64>("desired_rot", 1);
    rot_to_tag_pub_ = nh_.advertise<std_msgs::Float64>("rot_to_tag", 1);
    tag_pitch_pub_ = nh_.advertise<std_msgs::Float64>("tag_pitch", 1);

    // TODO: Load initial gains from parameter server
    pid_x_.initParam("~/pid/x");
    pid_th_.initParam("~/pid/th");


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


    reset();

}

void DockingController::reset() {
    last_time_ = ros::Time::now();

    error_times_ = 0;
}

double DockingController::getDistanceToTag() {
    // Returns the distance to the tag in the z direction. i.e the distance straight forward in the camera frame.
    return tag_pose_.position.z;
}

double DockingController::getPitchComponent() {
    
    // Get the pitch of the tag
    double tag_roll, tag_pitch, tag_yaw;
    // tf::Quaternion quat_tf;
    // tf::quaternionMsgToTF(tag_pose_.orientation, quat_tf);
    // tf::Matrix3x3(quat_tf).getRPY(tag_roll, tag_pitch, tag_yaw);

    // tf2::getEulerYPR(tag_pose_.orientation,tag_yaw,tag_pitch,tag_roll);
    geometry_msgs::TransformStamped tag_camera_transform;
    tag_camera_transform = tf_buffer_.lookupTransform("realsense_camera","DOCK_BUNDLE",ros::Time(0));

    tf2::getEulerYPR(tag_camera_transform.transform.rotation, tag_yaw, tag_pitch, tag_roll);

    return tag_pitch;
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

    dist_to_tag = sqrt(tag_pose_.position.z*tag_pose_.position.z + tag_point_rel_wheel_base_.x*tag_point_rel_wheel_base_.x);
    d = dist_to_tag*cos(getRotationToTag()+pitch);
    
    return d;
}

double DockingController::getDesiredRotation() {
    double d, des_rot;
    d = getLateralComponent();
    
    des_rot = exp(6*abs(d)+0.02);

    // Cap totation at max pi/2
    if(des_rot > 0.4){
        des_rot = 0.4;
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
        if(tag.id[0] == 0 && tag.id[1] == 4 && tag.id[2] == 3 && tag.id[3] == 2 && tag.id[4] == 1) {
            geometry_msgs::PoseStamped pose_msg;
            visualization_msgs::Marker d_pose_msg, n_pose_msg;
            pose_msg.pose = tag.pose.pose.pose;
            pose_msg.header.frame_id = tag.pose.header.frame_id;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.header.seq = seq++;

            detection_pub_.publish(pose_msg);

            // =============================================================
            // Visualisation markers for debug
            // =============================================================
            geometry_msgs::Quaternion quat_msg = tf::createQuaternionMsgFromRollPitchYaw(0.0,-getPitchComponent(),0.0);
            d_pose_msg.header.frame_id = tag.pose.header.frame_id;
            d_pose_msg.header.stamp = ros::Time::now();
            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            d_pose_msg.ns = "d_distance";
            d_pose_msg.id = 0;
            // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
            d_pose_msg.type = visualization_msgs::Marker::ARROW;
            // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
            d_pose_msg.action = visualization_msgs::Marker::ADD;
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            d_pose_msg.pose.position.x = -camera_to_wheelbase_transform.x;
            d_pose_msg.pose.position.y = 0;
            d_pose_msg.pose.position.z = -camera_to_wheelbase_transform.z;
            d_pose_msg.pose.orientation = quat_msg;
            
            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            d_pose_msg.scale.x = getLateralComponent();
            d_pose_msg.scale.y = 0.1;
            d_pose_msg.scale.z = 0.1;
            // Set the color -- be sure to set alpha to something non-zero!
            d_pose_msg.color.r = 0.0f;
            d_pose_msg.color.g = 1.0f;
            d_pose_msg.color.b = 0.0f;
            d_pose_msg.color.a = 1.0;
            d_pose_msg.lifetime = ros::Duration();

            d_pub_.publish(d_pose_msg);

            quat_msg = tf::createQuaternionMsgFromRollPitchYaw(0.0, 3.1415, getPitchComponent());
            n_pose_msg.header.frame_id = "chassis_link";
            n_pose_msg.header.stamp = ros::Time::now();
            // Set the namespace and id for this marker.  This serves to create a unique ID
            // Any marker sent with the same namespace and id will overwrite the old one
            n_pose_msg.ns = "d_distance";
            n_pose_msg.id = 0;
            // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
            n_pose_msg.type = visualization_msgs::Marker::ARROW;
            // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
            n_pose_msg.action = visualization_msgs::Marker::ADD;
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            n_pose_msg.pose.position.x = -camera_to_wheelbase_transform.z;
            n_pose_msg.pose.position.y = getLateralComponent();
            n_pose_msg.pose.position.z = 0;
            n_pose_msg.pose.orientation = quat_msg;

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            n_pose_msg.scale.x = getDistAlongTagNorm();
            n_pose_msg.scale.y = 0.1;
            n_pose_msg.scale.z = 0.1;
            // Set the color -- be sure to set alpha to something non-zero!
            n_pose_msg.color.r = 0.0f;
            n_pose_msg.color.g = 1.0f;
            n_pose_msg.color.b = 0.0f;
            n_pose_msg.color.a = 1.0;
            n_pose_msg.lifetime = ros::Duration();

            n_pub_.publish(n_pose_msg);


            std_msgs::Float64 f;
            f.data = getDesiredRotation();
            desired_rot_pub_.publish(f);
            
            f.data = getRotationToTag();
            rot_to_tag_pub_.publish(f);

            f.data = getPitchComponent();
            tag_pitch_pub_.publish(f);
            //======================================================================================


            tag_visible_ = true;

            // Save tag pose to tag_pose_
            // For tag_pose_.position:
            // z is depth of tag in camera frame. Higher value equals further away.
            // x is position of tag left and right of screen center. Right side is positive, left is degative.
            // y is position of tag  upp and down.
            tag_pose_ = tag.pose.pose.pose;

            tag_point_rel_wheel_base_.x = tag_pose_.position.x + camera_to_wheelbase_transform.x;
            tag_point_rel_wheel_base_.y = tag_pose_.position.y + camera_to_wheelbase_transform.y;
            tag_point_rel_wheel_base_.z = tag_pose_.position.z + camera_to_wheelbase_transform.z;



            geometry_msgs::TransformStamped tag_camera_transform;
            tf2::Vector3 vec_to_tag, cam_norm;
            double ang;
            tag_camera_transform = tf_buffer_.lookupTransform("chassis_link","DOCK_BUNDLE",ros::Time(0));
            vec_to_tag.setValue(tag_camera_transform.transform.translation.x,tag_camera_transform.transform.translation.y,tag_camera_transform.transform.translation.z);
            cam_norm = {0.0, 0.0, 1.0};
            ang = tf2::tf2Angle(cam_norm,vec_to_tag);


            ROS_INFO("pitch: %+.2f, rot_to_tag: %+.2f, ang: %+.2f, z: %+.2f, x: %+.2f d: %.2f, nz: %.2f", 
            getPitchComponent(), getRotationToTag(), ang, tag_pose_.position.z, tag_pose_.position.x, getLateralComponent(), getDistAlongTagNorm());

            break;
        }
    }
}
bool DockingController::computeVelocity(geometry_msgs::Twist& msg_out) {
    double des_rot;

    if(tag_visible_ == false) {
        msg_out.linear.x = 0.0;
        msg_out.angular.z = 0.0;
        return false;
    }

    if(state_ == DOCKING) {
        ros::Time current_time = ros::Time::now();

        // Calculate x, y and theta errors
        err_x_ = desired_offset_.x - getDistAlongTagNorm();
        err_y_ = desired_offset_.y - getLateralComponent();
        err_th_ = desired_offset_.z - getPitchComponent();

        msg_out.linear.x = pid_x_.computeCommand(desired_offset_.x - getDistAlongTagNorm(), current_time - last_time_);
        msg_out.angular.z = pid_th_.computeCommand(getDesiredRotation() - getRotationToTag(), current_time - last_time_);

        last_time_ = current_time;
        //ROS_INFO("err_x_: %.2f, err_y_: %.2f, err_th_: %.2f, des_rot: %.2f, norm_x: %.2f", err_x_, err_y_, err_th_, getDesiredRotation(), getDistAlongTagNorm());
    }

    return false;
}
