#include <unicorn_docking/docking_controller.h>

DockingController::DockingController() : nh_("~"), state_(DockingController::DockState::IDLE) {

    apriltag_sub_ = nh_.subscribe("/tag_detections", 100, &DockingController::apriltagDetectionsCb, this);
    state_sub_ = nh_.subscribe("state", 1, &DockingController::stateCb, this);

    // TODO: Load initial gains from parameter server
    pid_x_.initParam("~/pid/x");
    pid_th_.initParam("~/pid/th");

    //pid_x_.initPid(6.0, 1.0, 2.0, 0.3, -0.3, nh_);
    //pid_th_.initPid(6.0, 1.0, 2.0, 0.3, -0.3, nh2_);

    last_time_ = ros::Time::now();

}

double DockingController::getDistanceToTag() {
    double dist;
    dist = sqrt(  tag_pose_.position.x*tag_pose_.position.x
                + tag_pose_.position.y*tag_pose_.position.y);

    // Pos relative to tag
    // WHAT HAPPENS IF TAG NOT VISIBLE?!?!?!?!????? use odom to calculate??
    // WHAT IF OBJECT IN WAY???!?!?!?!????!??????
    return dist;
}

double DockingController::getRotationToTag() {
    double roll, pitch, yaw;
    tf::Quaternion quat_tf;
    tf::quaternionMsgToTF(tag_pose_.orientation, quat_tf);
    tf::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

    return yaw;
}

void DockingController::apriltagDetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    for(auto& tag : msg->detections) {
        if(tag.id[0] == 0) {
            tag_pose_ = tag.pose.pose.pose;
            break;
        }
    }
}

void DockingController::stateCb(const std_msgs::Int32::ConstPtr& msg) {
    setState((DockingController::DockState)msg->data);
}

geometry_msgs::Twist DockingController::computeVelocity() {
    geometry_msgs::Twist msg;

    if(state_ == DOCKING) {
        ros::Time current_time = ros::Time::now();

        msg.linear.x = pid_x_.computeCommand(desired_offset_.x - getDistanceToTag(), current_time - last_time_);

        // TODO: make actual angle offset for pitch, most likely zero offset, so we dont use z offset.
        msg.angular.z = pid_th_.computeCommand(desired_offset_.z - getRotationToTag(), current_time - last_time_);

        last_time_ = current_time;
    }
    return msg;
}
