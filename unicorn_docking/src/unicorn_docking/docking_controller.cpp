#include <unicorn_docking/docking_controller.h>

DockingController::DockingController() : nh_("~"), state_(DockingController::DockState::IDLE), tf_listener_(tf_buffer_) {

    apriltag_sub_ = nh_.subscribe("/tag_detections", 100, &DockingController::apriltagDetectionsCb, this);
    front_lidar_sub_ = nh_.subscribe("/frontLidar/scan", 1, &DockingController::frontLidarCb, this);

    d_pub_ = nh_.advertise<visualization_msgs::Marker>("d_tag", 1);
    n_pub_ = nh_.advertise<visualization_msgs::Marker>("n_tag", 1);
    desired_rot_pub_ = nh_.advertise<std_msgs::Float64>("desired_rot", 1);
    rot_to_tag_pub_ = nh_.advertise<std_msgs::Float64>("rot_to_tag", 1);
    tag_pitch_pub_ = nh_.advertise<std_msgs::Float64>("tag_pitch", 1);

    // TODO: Load initial gains from parameter server
    pid_x_.initParam("~/pid/x");
    pid_th_.initParam("~/pid/th");

    nh_.param("base_link_frame", base_link_frame_, std::string("chassis_link"));

    nh_.param("use_lidar", use_lidar_, true);

    nh_.param("front_lidar_angle", front_lidar_angle_, 1.57079632679);
    nh_.param("rear_lidar_angle", rear_lidar_angle_, 1.57079632679);
    nh_.param("max_time_since_lidar_scan", max_time_since_lidar_scan_, 1.0);

    if(use_lidar_) {
        // Get offset between lidar and base link

        nh_.param("lidar_frame", lidar_frame_, std::string("rear_laser"));

        lidar_sub_ = nh_.subscribe("/rearLidar/scan", 1, &DockingController::rearLidarCb, this);
        nh_.param("included_lidar_measures", lidar_indices_, std::vector<int>());

        while(ros::ok()) {
            try {
                geometry_msgs::TransformStamped lidar_transform = tf_buffer_.lookupTransform(base_link_frame_, lidar_frame_, ros::Time::now(), ros::Duration(max_tf_lookup_time_));

                lidar_offset_x_ = fabs(lidar_transform.transform.translation.x);


                break;
            } catch (tf2::TransformException& ex) {
                ROS_WARN("Could not find transform from %s to %s, %s", base_link_frame_.c_str(), lidar_frame_.c_str(), ex.what());
            }
        }
    }

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

    //return tag_pitch;
    return fuseAngles(tag_pitch, lidar_angle_, getDistAlongTagNorm());
}

double DockingController::getLateralComponent() {
    // Gets the distance perpendicular from the tag normal to the robot. (along tags x axis)
    
    return wheelbase_to_tag_tf_.transform.translation.x;
}

double DockingController::getDistAlongTagNorm() {
    // Gets the distance to the robot along the tag normal. (along tags z axis)
    //return wheelbase_to_tag_tf_.transform.translation.z;
    return fuseDistances(wheelbase_to_tag_tf_.transform.translation.z, lidar_dist_, lidar_angle_);
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

double DockingController::fuseDistances(double apriltag_dist, double lidar_dist, double lidar_angle) {
    // TODO: Make it possible to set min dist to activate lidar manually
    if(use_lidar_ && apriltag_dist < 0.8f && lidar_angle < 0.00872) {
        float lidar_contrib = 1.0 / exp(40 * (apriltag_dist - desired_offset_.x));

        if(lidar_contrib > 1.0)
            lidar_contrib = 1.0;

        return (1.0 - lidar_contrib) * apriltag_dist + lidar_contrib * lidar_dist;
    }
    else {
        return apriltag_dist;
    }
}

double DockingController::fuseAngles(double apriltag_angle, double lidar_angle, double apriltag_dist) {
    // TODO: Make it possible to set min dist to activate lidar manually
    if(use_lidar_ && apriltag_dist < 0.8f) {
        float lidar_contrib = 1.0 / exp(40 * (apriltag_dist - desired_offset_.x));

        if(lidar_contrib > 1.0)
            lidar_contrib = 1.0;

        return (1.0 - lidar_contrib) * apriltag_angle + lidar_contrib * lidar_angle;
    }
    else {
        return apriltag_angle;
    }

}

double DockingController::getDistanceToClosestObject(const sensor_msgs::LaserScan& laser_scan, double laser_scan_angle) {
    float range_min = 0.0f, current_angle = 0.0f;
    // TODO: check time of laser scan so it is not to old.

    range_min = laser_scan.range_max;
    for( int i; i < laser_scan.ranges.size(); i++ ) {
        // Get the angle of the laser scan
        current_angle = laser_scan.angle_min + i*laser_scan.angle_increment;
        if((current_angle >= -laser_scan_angle/2) && (current_angle >= laser_scan_angle/2)) {
            if(laser_scan.ranges[i] < range_min) {
                range_min = laser_scan.ranges[i];
            }
        }
    }
    return range_min;
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

void DockingController::rearLidarCb(const sensor_msgs::LaserScanConstPtr& msg) {
    float tempX = 0.0f, avgX = 0.0f, avgY = 0.0f, num = 0.0f, denom = 0.0f;
    rear_lidar_scan_ = *msg;

    // Calculate angle and distance to wall
    for(int i : lidar_indices_) {
        tempX = msg->angle_min + i * msg->angle_increment;
        avgX += tempX;
        avgY += msg->ranges[i]*cos(tempX);
    }

    avgY /= lidar_indices_.size();
    avgX /= lidar_indices_.size();

    float x, y;
    for(int i : lidar_indices_) {
        x = i * msg->angle_increment + msg->angle_min;
        y = msg->ranges[i] * cos(x);
        num += (x - avgX) * (y - avgY);
        denom += (x - avgX) * (x - avgX);
    }

    if(denom == 0.0)
        return;

    double res = num / denom;

    lidar_dist_ = avgY + lidar_offset_x_;
    lidar_angle_ = atan(res);

}

void DockingController::frontLidarCb(const sensor_msgs::LaserScanConstPtr& msg) {
    front_lidar_scan_ = *msg;
}

bool DockingController::computeVelocity(geometry_msgs::Twist& msg_out) {

    ros::Time current_time = ros::Time::now();
    double speed_multiplier, time_since_lidar_scan;
    bool got_error = false;
    
    if(state_ == DOCKING) {

        // Calculate x, y and theta errors
        err_x_ = desired_offset_.x - getDistAlongTagNorm();
        err_y_ = desired_offset_.y - getLateralComponent();
        err_th_ = desired_offset_.z - getPitchComponent();

        // Try to timetravel!
        // Try to find transform between DOCK_BUNDLE frame at tag_last_seen_ time and chassis_link at current_time
        try {
            // The -0.2 on max_tf_lookup_time_ is to allow "rotation_to_tag = getRotationToTag()" below to finnish succesfully before max_tf_lookup_time_ have passed
            wheelbase_to_tag_tf_ = tf_buffer_.lookupTransform("DOCK_BUNDLE", tag_last_seen_, base_link_frame_, current_time, "map", ros::Duration(max_tf_lookup_time_-0.2));
        }
        catch(tf2::TransformException &ex) {
            // If no transform could be found, set linear and angular velocities to zero
            ROS_WARN("in computeVelocity(): %s",ex.what());
            msg_out.linear.x = 0.0;
            msg_out.angular.z = 0.0;
            return false;
        }
        
        // call getRotationToTag as it does a lookupTransform. This helps prevent max_tf_lookup_time_ to have passed even tho the lookupTransform above succeded 
        double rotation_to_tag = getRotationToTag();
        // If rotation to tag failed it returns -100
        if(rotation_to_tag == -100) {
            ROS_WARN("getRotationToTag() did not succed. Setting velocities to 0");
            got_error = true;
        }

        // Check if rear lidar scan is to old
        time_since_lidar_scan = (current_time - rear_lidar_scan_.header.stamp).toSec();
        if(got_error == false && time_since_lidar_scan > max_time_since_lidar_scan_) {
            ROS_INFO("rear lidar scan is older than %f seconds, it is: %f seconds old", max_time_since_lidar_scan_, time_since_lidar_scan);
            got_error = true;
        }

        // Check if front lidar scan is to old
        time_since_lidar_scan = (current_time - front_lidar_scan_.header.stamp).toSec();
        if(got_error == false && time_since_lidar_scan > max_time_since_lidar_scan_) {
            ROS_INFO("front lidar scan is older than %f seconds, it is: %f seconds old", max_time_since_lidar_scan_, time_since_lidar_scan);
            got_error = true;
        }

        // If an error occured set velocities to 0.0
        if(got_error){
            msg_out.linear.x = 0.0;
            msg_out.angular.z = 0.0;
            return false;
        }



        msg_out.linear.x = pid_x_.computeCommand(desired_offset_.x - getDistAlongTagNorm(), current_time - last_time_);

        // when x is positive, the robot is moving forward
        if(msg_out.linear.x >= 0) {
            // If linear.x is positive the rotation needs to be the other way as it is moving away from the tag and not towards it.
            msg_out.angular.z = pid_th_.computeCommand(-getDesiredRotation() - rotation_to_tag, current_time - last_time_);

            // Get speed multiplier depending on distance to objects infront of robot
            speed_multiplier = 2.5*(getDistanceToClosestObject(front_lidar_scan_,front_lidar_angle_)-min_distance_infront_);
        }
        else {
            // robot is backing towards tag
            msg_out.angular.z = pid_th_.computeCommand(getDesiredRotation() - rotation_to_tag, current_time - last_time_);

            // Get speed multiplier depending on distance to objects behind robot
            speed_multiplier = 5*(getDistanceToClosestObject(rear_lidar_scan_,rear_lidar_angle_)-min_distance_behind_);
        }
        
        // Cap speed multiplier between 0 and 1
        if (speed_multiplier > 1) {
                speed_multiplier = 1;
        }
        else if(speed_multiplier < 0) {
            speed_multiplier = 0;

            // If speed multiplier is 0, do not rotate
            msg_out.angular.z = 0.0;
        }

        // Apply speed modifier
        msg_out.linear.x = speed_multiplier*msg_out.linear.x;


        last_time_ = current_time;
    }

    return false;
}
