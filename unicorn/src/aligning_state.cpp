#include "unicorn/aligning_state.h"

ALIGNINGState::ALIGNINGState()
{
    ROS_INFO("Created new ALIGNING State");
}

ALIGNINGState::~ALIGNINGState()
{
}

Command ALIGNINGState::run()
{
    ROS_INFO("Running ALIGNING state");
    // ros::Rate rate(50);
    // ROS_INFO("[unicorn_statemachine] Aligning with garbage disposal...");
    // sendGoal(refuse_bin_pose_.x + 1.5 * cos(refuse_bin_pose_.yaw), refuse_bin_pose_.y + 1.5 * sin(refuse_bin_pose_.yaw), refuse_bin_pose_.yaw);
    // updateAndPublishState(state_enum::ENTERING);
    // while (move_base_clt_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    // }
}
