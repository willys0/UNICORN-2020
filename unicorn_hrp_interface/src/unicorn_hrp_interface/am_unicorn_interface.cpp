// sends commands to the HRP in terms of speed, angular and linear

#include <unicorn_hrp_interface/am_unicorn_interface.h>

#include <std_srvs/SetBool.h>
#include <actionlib_msgs/GoalID.h>
#include <am_driver/Mode.h>
#include <std_msgs/UInt16.h>

AmUnicornInterface::AmUnicornInterface()
{
  cmd_vel_sub_ = n_.subscribe("/unicorn/cmd_vel", 0, &AmUnicornInterface::cmdVelCallback, this);
  unicorn_cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 0);

  hrp_status_sub_ = n_.subscribe("/sensor_status", 0, &AmUnicornInterface::amStatusCallback, this);
  hrp_mode_pub_ = n_.advertise<std_msgs::UInt16>("/cmd_mode", 1, true);

  movebase_cancel_pub_ = n_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
  dock_cancel_pub_ = n_.advertise<actionlib_msgs::GoalID>("/dock/cancel", 1);
  lift_cancel_pub_ = n_.advertise<actionlib_msgs::GoalID>("/lift/lift_action/cancel", 1);

  n_.param("max_velocity_delay", max_velocity_delay_, 0.1f);

  last_cmd_vel_time_ = ros::Time::now();
}

void AmUnicornInterface::cmdVelCallback(const geometry_msgs::Twist& cmd_vel)
{
  unicorn_cmd_vel_ = cmd_vel;
  unicorn_cmd_vel_.linear.x = -unicorn_cmd_vel_.linear.x;
  unicorn_cmd_vel_.linear.z = -unicorn_cmd_vel_.linear.z;

  last_cmd_vel_time_ = ros::Time::now();
}

void AmUnicornInterface::amStatusCallback(const am_driver::SensorStatusConstPtr& msg) {
  static std_srvs::SetBool srv;
  static actionlib_msgs::GoalID cancel_msg;
  static std_msgs::UInt16 mode_msg;

  if(msg->mowerInternalState != am_driver::SensorStatus::MOWER_INTERNAL_STATUS_PAUSED) {
    srv.request.data = true;
    ros::service::call("/unicorn_state_machine_node/force_stop", srv);

    movebase_cancel_pub_.publish(cancel_msg);
    dock_cancel_pub_.publish(cancel_msg);
    lift_cancel_pub_.publish(cancel_msg);
  }

  if(msg->sensorStatus & am_driver::SensorStatus::SENSOR_STATUS_LOOP_ON) {
    mode_msg.data = am_driver::Mode::MODE_LOOP_OFF;
    hrp_mode_pub_.publish(mode_msg);
  }
}

void AmUnicornInterface::publishCmd()
{
  if( (ros::Time::now() - last_cmd_vel_time_).toSec() > max_velocity_delay_ ) {
    unicorn_cmd_vel_.linear.x *= 0.8f;
    unicorn_cmd_vel_.angular.z *= 0.8f;

    if(unicorn_cmd_vel_.linear.x < 0.1f && unicorn_cmd_vel_.linear.x > -0.1f)
      unicorn_cmd_vel_.linear.x = 0.0f;

    if(unicorn_cmd_vel_.angular.z < 0.1f && unicorn_cmd_vel_.angular.z > -0.1f)
      unicorn_cmd_vel_.angular.z = 0.0f;

  }

  unicorn_cmd_vel_pub_.publish(unicorn_cmd_vel_);
}
