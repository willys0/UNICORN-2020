// sends commands to the HRP in terms of speed, angular and linear

#include <unicorn/am_unicorn_interface.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "am_unicorn_interface");
  ROS_INFO("Started automower to unicorn interface");
  AmUnicornInterface am_unicorn_interface;
  ros::Rate r(30.0);
  while(ros::ok())
  {
    am_unicorn_interface.publishCmd();
    ros::spinOnce();
    r.sleep();
  }
}

AmUnicornInterface::AmUnicornInterface()
{
  cmd_vel_sub_ = n_.subscribe("/unicorn/cmd_vel", 0, &AmUnicornInterface::cmdVelCallback, this);
  unicorn_cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 0);
}

void AmUnicornInterface::cmdVelCallback(const geometry_msgs::Twist& cmd_vel)
{
  unicorn_cmd_vel_ = cmd_vel;
  unicorn_cmd_vel_.linear.x = -unicorn_cmd_vel_.linear.x;
  unicorn_cmd_vel_.linear.z = -unicorn_cmd_vel_.linear.z;
}

void AmUnicornInterface::publishCmd()
{
  unicorn_cmd_vel_pub_.publish(unicorn_cmd_vel_);
}

