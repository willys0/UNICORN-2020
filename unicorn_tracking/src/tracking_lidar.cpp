#include "tracking_lidar.h"


static geometry_msgs::TransformStamped transform_frames;
int main(int argc, char** argv){
  ros::init(argc, argv, "tracking_lidar");

  ros::NodeHandle nh;
  ROS_INFO("Started Lidar Tracking node");
  tracting_lidar tracting_lidar_interface;
  ros::Rate r(30.0);


  //tf2_ros::Buffer tf_buffer;
  //tf2_ros::TransformListener tf2_listener(tf_buffer);
  //transform_frames = tf_buffer.lookupTransform("chassis_link", "map", ros::Time(0), ros::Duration(1.0) );

  while(ros::ok())
  {
    //ROS_INFO("Tracking node running");
    ros::spinOnce();
    r.sleep();
  }
}


tracting_lidar::tracting_lidar()
{
  odometry_sub_ = n_.subscribe("/odometry/filtered", 10, &tracting_lidar::odomCallback, this);
  map_sub_ = n_.subscribe("/map", 10, &tracting_lidar::mapCallback, this);
  scan_sub_ = n_.subscribe("/frontLidar/scan", 10, &tracting_lidar::scanCallback, this);
  //odometry_transform_pub_ = n_.advertise<nav_msgs::Odometry>("/wheel_encoder/odom_transformed", 10,true);
}

void tracting_lidar::odomCallback(const nav_msgs::Odometry& odometry)
{
    ROS_INFO("New odom Message!");
    odometry_data_ = odometry;
}

void tracting_lidar::mapCallback(const nav_msgs::OccupancyGrid& map)
{
    ROS_INFO("New map Message!");
    map_data_ = map;
}

void tracting_lidar::scanCallback(const sensor_msgs::LaserScan& scan)
{
    ROS_INFO("New scan Message!");
    scan_data_ = scan;
    // Do stuff here 
}


/*
static geometry_msgs::TransformStamped transform_frames;
int main(int argc, char** argv){
  ros::init(argc, argv, "tracking_lidar");
  ROS_INFO("Started Lidar Tracking node");
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

/*
odom_transform::odom_transform()
{
  odometry_sub_ = n_.subscribe("/wheel_encoder/odom", 10, &odom_transform::msgCallback, this);
  odometry_transform_pub_ = n_.advertise<nav_msgs::Odometry>("/wheel_encoder/odom_transformed", 10,true);
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
*/