#include <unicorn_roborio_bridge/lift_interface.h>

LiftInterface::LiftInterface(ros::NodeHandle nh) :
    nh_(nh)
{
    initLiftMsg();

    lift_state_pub_ = nh_.advertise<std_msgs::Int32>("/lift/state", 1);
    run_lift_pub_   = nh_.advertise<std_msgs::Int32>("/lift/picking_routine", 1);

}

void LiftInterface::publish() {
    lift_state_pub_.publish(lift_state_);
}

void LiftInterface::startTimer() {
    lift_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / lift_state_pub_freq_), &LiftInterface::liftPubTimeout, this);
}

void LiftInterface::startPickupRoutine() {
    run_msg_.data = RioLiftAction::PICKUP;
    run_lift_pub_.publish(run_msg_);
    waitForMsgPublish();
}

void LiftInterface::startDropoffRoutine() {
    run_msg_.data = RioLiftAction::DROPOFF;
    run_lift_pub_.publish(run_msg_);
    waitForMsgPublish();
}

void LiftInterface::stopLift() {
    run_msg_.data = RioLiftAction::STOP;
    run_lift_pub_.publish(run_msg_);
    waitForMsgPublish();
}

void LiftInterface::cancelLift() {
    run_msg_.data = RioLiftAction::CANCEL;
    run_lift_pub_.publish(run_msg_);
    waitForMsgPublish();
}

LiftInterface::LiftState LiftInterface::getCurrentState() {
    switch(lift_state_.data) {
        case 0:
            return LiftState::IDLE;
        case 1:
        case 2:
            return LiftState::RUNNING;
        case 3:
            return LiftState::RESETTING;
        default:
            return LiftState::ERROR;
    }
}
void LiftInterface::initLiftMsg() {
    run_msg_.data = 0;

    nh_.param<float>("lift/publish_frequency", lift_state_pub_freq_, 10.0);

    ROS_INFO("[Lift settings] publish_frequency: %.2f",
        lift_state_pub_freq_);
}

void LiftInterface::liftPubTimeout(const ros::TimerEvent& e) {
    publish();
}

void LiftInterface::waitForMsgPublish() {
    ros::spinOnce();
    ros::Duration(1).sleep();
}
