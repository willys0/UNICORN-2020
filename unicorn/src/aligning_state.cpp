#include "aligning_state.h"
AligningState::AligningState()
{
}

AligningState::~AligningState()
{
}

cmd_struct_ AligningState::run()
{
    ros::Rate rate(50);
    ROS_INFO("[unicorn_statemachine] Aligning with garbage disposal...");
    sendGoal(refuse_bin_pose_.x + 1.5 * cos(refuse_bin_pose_.yaw), refuse_bin_pose_.y + 1.5 * sin(refuse_bin_pose_.yaw), refuse_bin_pose_.yaw);
    updateAndPublishState(current_state::ENTERING);
    while (move_base_clt_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ros::spinOnce();
        rate.sleep();
    }
}
