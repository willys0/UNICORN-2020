// transforms the odometry data from the wheel encoders

#include <unicorn/odometry_transform.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

static geometry_msgs::TransformStamped transform_frames;
int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_transform");
  ROS_INFO("Started odometry transform node");
  odom_transform odom_transform_interface;
  ros::Rate r(100.0);


  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener(tf_buffer);
  transform_frames = tf_buffer.lookupTransform("chassis_link", "lift_link", ros::Time(0), ros::Duration(1.0) );
  while(ros::ok())
  {
    odom_transform_interface.publishmsg();
    ros::spinOnce();
    r.sleep();
  }
}

odom_transform::odom_transform()
{
  odometry_sub_ = n_.subscribe("/wheel_encoder/odom", 0, &odom_transform::msgCallback, this);
  odometry_transform_pub_ = n_.advertise<nav_msgs::Odometry>("/wheel_encoder/odom_transformed", 0);
}

void odom_transform::msgCallback(const nav_msgs::Odometry& odometry)
{
  odometry_data_ = odometry;
  tf2::doTransform((odometry.pose.pose), (odometry_data_.pose.pose), transform_frames);
}

void odom_transform::publishmsg()
{
  odometry_transform_pub_.publish(odometry_data_);
}


