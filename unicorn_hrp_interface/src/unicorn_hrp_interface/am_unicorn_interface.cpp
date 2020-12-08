// sends commands to the HRP in terms of speed, angular and linear

#include <unicorn_hrp_interface/am_unicorn_interface.h>

AmUnicornInterface::AmUnicornInterface()
{
  cmd_vel_sub_ = n_.subscribe("/unicorn/cmd_vel", 0, &AmUnicornInterface::cmdVelCallback, this);
  unicorn_cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 0);
}

void AmUnicornInterface::cmdVelCallback(const geometry_msgs::Twist& cmd_vel)
{
  unicorn_cmd_vel_ = cmd_vel;
  unicorn_cmd_vel_.linear.x = -unicorn_cmd_vel_.linear.x;
  unicorn_cmd_vel_.linear.z = -unicorn_cmd_vel_.linear.z;
}

void AmUnicornInterface::amStatusCallback(const am_driver::SensorStatusConstPtr& msg) {
  if(msg->mowerInternalState != am_driver::SensorStatus::MOWER_INTERNAL_STATUS_PAUSED) {

  }

}

void AmUnicornInterface::publishCmd()
{
  unicorn_cmd_vel_pub_.publish(unicorn_cmd_vel_);
}
