// sends commands to the HRP in terms of speed, angular and linear

#include <unicorn_hrp_interface/am_unicorn_interface.h>

AmUnicornInterface::AmUnicornInterface()
{
  cmd_vel_sub_ = n_.subscribe("/unicorn/cmd_vel", 0, &AmUnicornInterface::cmdVelCallback, this);
  unicorn_cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 0);

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
  if(msg->mowerInternalState != am_driver::SensorStatus::MOWER_INTERNAL_STATUS_PAUSED) {

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
