#include "tracking_lidar.h"



static bool map_received = false;
static int* adaptive_breaK_point(const sensor_msgs::LaserScan& scan);

int main(int argc, char** argv){
  ros::init(argc, argv, "tracking_lidar");

  ros::NodeHandle nh;
  ROS_INFO("Started Lidar Tracking node");
  tracting_lidar tracting_lidar_interface;
  ros::Rate r(30.0);


  ros::spin();
}


tracting_lidar::tracting_lidar()
: n_("~")
{

  odometry_sub_ = n_.subscribe("/odometry/filtered", 10, &tracting_lidar::odomCallback, this);
  map_sub_ = n_.subscribe("/map", 10, &tracting_lidar::mapCallback, this);
  scan_sub_ = n_.subscribe("/frontLidar/scan", 10, &tracting_lidar::scanCallback, this);
  
  n_.param("lambda",lambda, 0.15f);
  n_.param("Max_Laser_dist",max_dist_laser, 10);
  n_.param("Static_map_removal_tolerance",static_remove_dist, 4);
  n_.param("polygon_tolerance",polygon_tolerance, 1.04f);
  n_.param("polygon_min_points_required",polygon_min_points, 4);
  

  
  //odometry_transform_pub_ = n_.advertise<nav_msgs::Odometry>("/wheel_encoder/odom_transformed", 10,true);
}

void tracting_lidar::odomCallback(const nav_msgs::Odometry& odometry)
{
    //ROS_INFO("New odom Message!");
    odometry_data_ = odometry;
    
    tf::Quaternion q(odometry.pose.pose.orientation.x,odometry.pose.pose.orientation.y,odometry.pose.pose.orientation.z,odometry.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll,pitch,yaw);
    yaw = -yaw;

    x = odometry.pose.pose.position.x;
    y = odometry.pose.pose.position.y;
    z = odometry.pose.pose.position.z;

    //ROS_INFO("Orientation %lf  %lf %lf",roll,pitch,yaw);

}

void tracting_lidar::mapCallback(const nav_msgs::OccupancyGrid& map)
{
    //ROS_INFO("New map Message!");
    map_data_ = map;
    map_received = true;
    mapx = map_data_.info.height;
    mapy = map_data_.info.width;
}

void tracting_lidar::scanCallback(const sensor_msgs::LaserScan& scan)
{
    //ROS_INFO("New scan Message!");
    scan_data_ = scan;
    
    if(map_received)
    {
      adaptive_breaK_point(scan_data_);
      static_map_filter(map_data_);
      polygon_extraction();



    }

    
    // Do stuff here 
}



void tracting_lidar::adaptive_breaK_point(const sensor_msgs::LaserScan& scan)
{

  int i,j = 0;
  float current_angle,r,p,dmax, deg2rad = PI/180;
  int max_itterations = round((scan.angle_max - scan.angle_min)/scan.angle_increment);
  // ROS_INFO("New scan Message %d ",max_itterations);
  for(i = 0; i < max_itterations+1; i++)
  {
    current_angle = float(i)*scan.angle_increment+scan.angle_min;
    // Calculate xy position
    xy_positions[i][1] = (scan.ranges[i])*(cos(current_angle)*cos(yaw) + sin(current_angle)*sin(yaw)) + x;
    xy_positions[i][2] = (scan.ranges[i])*(sin(current_angle)*cos(yaw) - cos(current_angle)*sin(yaw)) + y;

    if(i == 0)
    {
      clusters[i] = 0;
    }else
    {
      r = sqrt(pow(xy_positions[i-1][1],2) + (xy_positions[i-1][2],2));
      p = sqrt(pow(xy_positions[i-1][1] - xy_positions[i][1],2) + pow(xy_positions[i-1][2] - xy_positions[i][2],2));
      dmax = r*sin(scan.angle_increment)/sin(lambda + scan.angle_increment) + 3*scan.angle_increment;
      if(dmax < p)
        j++;
      
      clusters[i] = j;
      
    }
    if(scan.ranges[i] > max_dist_laser)
      clusters[i] = -1;
    //ROS_INFO("New cluster %f %f %d",xy_positions[i][1],xy_positions[i][2],clusters[i]);

  }
}

void tracting_lidar::static_map_filter(const nav_msgs::OccupancyGrid& map)
{
  int i,m,n;
  uint32_t x_map,y_map;
  // uint8_t (*map_matrix)[4000] = reinterpret_cast<uint8_t (*)[4000]>(map.data);

  // ROS_INFO("-----------------------------------------------------------");
  for(i=0; i < 800;i++){

    x_map = round(xy_positions[i][1]/map.info.resolution) + (map.info.width/2);
    y_map = round(xy_positions[i][2]/map.info.resolution) + (map.info.height/2);
    /**/
    if(clusters[i] != -1)
    {
      for(m=-static_remove_dist; m <= static_remove_dist;m++)
        for(n=-static_remove_dist; n <= static_remove_dist;n++){

          if(map.data[(x_map+m) + (y_map+n-1)*map.info.width] > 0){
            clusters[i] = -1;
            m = static_remove_dist+1;
            n = static_remove_dist+1;
          }

        }

    }
   // ROS_INFO("New cluster %d %d %d %d",x_map,y_map,clusters[i],i);

  }
}



void tracting_lidar::polygon_extraction(){
  /**/
  int i, current_cluster = clusters[0];
  int start_point = 0, end_point;
  int polygon[800];
  memset(polygon, 0, 800*sizeof(polygon[0])); 

  for(i=1; i < 800; i++){
    if(clusters[i] != current_cluster){
      end_point = i-1;
      if(current_cluster != -1){

          ROS_INFO("New cluster %d %d %d",start_point,end_point,end_point-start_point+1);
          polygon[start_point] = 1;
          polygon[end_point] = 1;
          extract_corners(start_point,polygon,end_point-start_point+1);

      }
      current_cluster = clusters[i];
      start_point = i;
    }

  }
}

void tracting_lidar::extract_corners(int startpoint,int *polygon, int length)
{
  if(length < polygon_min_points)
    return;
  

  int max_itteration = ceil(log2(float(length)));
  float distance_start =  sqrt(pow(xy_positions[startpoint][1] - xy_positions[startpoint + length - 1][1],2) + pow(xy_positions[startpoint][2] - xy_positions[startpoint + length - 1][2],2));
  
  int *best_point = &startpoint;
  float *best_dist = &distance_start;

  search_longest(startpoint, startpoint+length-1,startpoint, length, distance_start, 1, max_itteration, best_point, best_dist);

  //ROS_INFO("halfway points %d",point_check);



}

void tracting_lidar::search_longest(int startpoint,int end_point, int current_point, int length, float distance_S, int itteration, int max_itteration, int *best_point, float *best_dist)
{

  if(length < 1 || (startpoint+length)>799)
    return; 
  
  float distance_1, distance_2, distance_total;
  int j = floor(float(length)/(2));
  int check_point = current_point + j-1;
  itteration++;


  distance_1 =  sqrt(pow(xy_positions[startpoint][1] - xy_positions[check_point][1],2) + pow(xy_positions[startpoint][2] - xy_positions[check_point][2],2));
  distance_2 =  sqrt(pow(xy_positions[check_point][1] - xy_positions[end_point][1],2) + pow(xy_positions[check_point][2] - xy_positions[end_point][2],2));
  distance_total = distance_1 + distance_2;

  ROS_INFO("check point %d %d %f %f",check_point, length,distance_S, distance_total);
  if(distance_total > *best_dist){
    *best_point = check_point;
    *best_dist = distance_total;
  }

  if(itteration < max_itteration)
    if(distance_total > distance_S*0.9)
    {
      if(distance_total < distance_S)
        distance_total = distance_S;
      //ROS_INFO("check point %d %d -- %d %d",startpoint,check_point - startpoint + 1,check_point,startpoint+length - check_point);
      search_longest(startpoint,end_point, startpoint, check_point - startpoint + 1, distance_total, itteration, max_itteration, best_point, best_dist);
      search_longest(startpoint,end_point, check_point, startpoint+length - check_point, distance_total, itteration, max_itteration, best_point, best_dist);
      



    }


  /*
  int i,j;
  float distance_1, distance_2, distance_total;

  int check_point = startpoint + ceil(float(length)/2)-1;
  int max_itteration = ceil(log2(float(length)));
  float distance_start =  sqrt(pow(xy_positions[startpoint][1] - xy_positions[startpoint + length - 1][1],2) + pow(xy_positions[startpoint][2] - xy_positions[startpoint + length - 1][2],2));

  distance_1 =  sqrt(pow(xy_positions[startpoint][1] - xy_positions[check_point][1],2) + pow(xy_positions[startpoint][2] - xy_positions[check_point][2],2));
  distance_2 =  sqrt(pow(xy_positions[check_point][1] - xy_positions[startpoint + length - 1][1],2) + pow(xy_positions[check_point][2] - xy_positions[startpoint + length - 1][2],2));
  distance_total = distance_1 + distance_2;

  for(i=2; i < max_itteration; i++)
  {
    


    if(distance_start*polygon_tolerance < distance_total)
    {
      j = floor(float(length)/pow(2,i));



    }else{
      return check_point;
    }

  }*/



  





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