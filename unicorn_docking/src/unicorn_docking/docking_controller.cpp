#include <unicorn_docking/docking_controller.h>

DockingController::DockingController() : nh_() {

    apriltag_sub_ = nh_.subscribe("/tag_detections", 100, &DockingController::apriltagDetectionsCb, this);

    // TODO: Load initial gains from parameter server
    pid_x_.initPid(6.0, 1.0, 2.0, 0.3, -0.3, nh_);

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

void DockingController::apriltagDetectionsCb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg) {
    for(auto& tag : msg->detections) {
        if(tag.id[0] == 0) {
            tag_pose_ = tag.pose.pose.pose;
            break;
        }
    }
}

geometry_msgs::Twist DockingController::computeVelocity() {
    geometry_msgs::Twist msg;

    ros::Time current_time = ros::Time::now();

    msg.linear.x = pid_x_.computeCommand(desired_offset_.x - getDistanceToTag(), current_time - last_time_);

    last_time_ = current_time;

    return msg;
}
