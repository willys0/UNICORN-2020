#include "tracking_lidar.h"



static bool map_received = false;
static bool odom_received = false;
static bool scan_received = false;


int main(int argc, char** argv){
  ros::init(argc, argv, "tracking_lidar");


  ROS_INFO("Started Lidar Tracking node");
  tracking_lidar tracking_lidar_interface;
  ros::Rate r(5.0);

/*
while(ros::ok)
  {


    ros::spinOnce();
    r.sleep();
    //ROS_INFO(" Running ");
    
    if(map_received && odom_received && scan_received){
      ROS_INFO(" doing shit ");
      tracking_lidar_interface.adaptive_breaK_point();
      tracking_lidar_interface.static_map_filter();
      tracking_lidar_interface.polygon_extraction();
      scan_received = false;

    }
    



    }*/
    ros::spin();
}


tracking_lidar::tracking_lidar()
: n_("~")
{

  odometry_sub_ = n_.subscribe("/odometry/filtered", 10, &tracking_lidar::odomCallback, this);
  map_sub_ = n_.subscribe("/map", 10, &tracking_lidar::mapCallback, this);
  scan_sub_ = n_.subscribe("/frontLidar/scan", 10, &tracking_lidar::scanCallback, this);
  
  n_.param("lambda",lambda, 0.15f);
  n_.param("Max_Laser_dist",max_dist_laser, 10);
  n_.param("Static_map_removal_tolerance",static_remove_dist, 4);
  n_.param("polygon_tolerance",polygon_tolerance, 1.04f);
  n_.param("polygon_min_points_required",polygon_min_points, 4);



  memset(polygon_size, 0, MAX_OBJECTS*sizeof(polygon_size[0])); 

  
  //odometry_transform_pub_ = n_.advertise<nav_msgs::Odometry>("/wheel_encoder/odom_transformed", 10,true);
}

void tracking_lidar::odomCallback(const nav_msgs::Odometry& odometry)
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

    odom_received = true;

    //ROS_INFO("Orientation %lf  %lf %lf",roll,pitch,yaw);

}

void tracking_lidar::mapCallback(const nav_msgs::OccupancyGrid& map)
{
    //ROS_INFO("New map Message!");
    map_data_ = map;
    map_received = true;
    mapx = map_data_.info.height;
    mapy = map_data_.info.width;
}

void tracking_lidar::scanCallback(const sensor_msgs::LaserScan& scan)
{
    //ROS_INFO("New scan Message!");
    scan_data_ = scan;
    scan_received = true;
    
    
    if(map_received && odom_received && scan_received){

      adaptive_breaK_point();
      static_map_filter();
      polygon_extraction();
      polygon_attribute_extraction();
      scan_received = false;

    }
    
    // Do stuff here 
}



void tracking_lidar::adaptive_breaK_point()
{

  int i,j = 0;
  float current_angle,r,p,dmax, deg2rad = PI/180;
  int max_itterations = round((scan_data_.angle_max - scan_data_.angle_min)/scan_data_.angle_increment);
  for(i = 0; i < max_itterations+1; i++)
  {
    current_angle = float(i)*scan_data_.angle_increment+scan_data_.angle_min;
    // Calculate xy position
    xy_positions[i][1] = (scan_data_.ranges[i])*(cos(current_angle)*cos(yaw) + sin(current_angle)*sin(yaw)) + x;
    xy_positions[i][2] = (scan_data_.ranges[i])*(sin(current_angle)*cos(yaw) - cos(current_angle)*sin(yaw)) + y;

    if(i == 0)
    {
      clusters[i] = 0;
    }else
    {
      r = sqrt(pow(xy_positions[i-1][1],2) + (xy_positions[i-1][2],2));
      p = sqrt(pow(xy_positions[i-1][1] - xy_positions[i][1],2) + pow(xy_positions[i-1][2] - xy_positions[i][2],2));
      dmax = r*sin(scan_data_.angle_increment)/sin(lambda + scan_data_.angle_increment) + 3*scan_data_.angle_increment;
      if(dmax < p)
        j++;
      
      clusters[i] = j;
      
    }
    if(scan_data_.ranges[i] > max_dist_laser)
      clusters[i] = -1;
    //ROS_INFO("New cluster %f %f %d",xy_positions[i][1],xy_positions[i][2],clusters[i]);

  }
}

/* Filters static map from scan, requires reliable odometry and static map*/
void tracking_lidar::static_map_filter()
{
  int i,m,n;
  uint32_t x_map,y_map;

  for(i=0; i < 800;i++){

    x_map = round(xy_positions[i][1]/map_data_.info.resolution) + (map_data_.info.width/2);
    y_map = round(xy_positions[i][2]/map_data_.info.resolution) + (map_data_.info.height/2);

    if(clusters[i] != -1)
    {
      for(m=-static_remove_dist; m <= static_remove_dist;m++)
        for(n=-static_remove_dist; n <= static_remove_dist;n++){

          if(map_data_.data[(x_map+m) + (y_map+n-1)*map_data_.info.width] > 0){
            clusters[i] = -1;
            m = static_remove_dist+1;
            n = static_remove_dist+1;
          }

        }

    }
  }
}

/* Extracts polygon shapes from lidar clusters*/
void tracking_lidar::polygon_extraction(){
  /**/
  int i,j, current_cluster = clusters[0], m = 0,n,l;
  int start_point = 0, end_point;
  geometry_msgs::Point32 point;

  // Clear shape array
  /**/
  for(i=0; i < MAX_OBJECTS; i++)
  {
    if(polygon_size[i] > 0)
    {
      // ROS_INFO("clear cluster %d size %d",i,polygon_size[i]);
      /**/
      shapes[i].points.clear();
      polygon_size[i] = 0;

    }

  }

  for(i=1; i < 800; i++){
    polygon[i] = 0;
    if(clusters[i] != current_cluster){ 
      end_point = i-1;
      if(current_cluster != -1){
          l = end_point-start_point+1;
          if(l >= 3){

            polygon[start_point] = 1;
            polygon_size[m]++;
            polygon[end_point] = 1;
            polygon_size[m]++;
            

            extract_corners(start_point,end_point,end_point-start_point+1,m);
            
            for(j = start_point;j <= end_point; j++)
            {
               
              if(polygon[j])
              {
               
                
                point.x = xy_positions[j][1];
                point.y = xy_positions[j][2];
                point.z = 0;
                shapes[m].points.push_back(point);
     
                
              }
            }
            // ROS_INFO("New cluster %d cluster %d",polygon_size[m],m);
            m++;
          }
      }
      current_cluster = clusters[i];
      start_point = i;
    }

  }
}

/*  Finds all corners in a scan between two points*/
void tracking_lidar::extract_corners(int startpoint,int endpoint, int length,int shape_nr)
{
  if(length < polygon_min_points)
    return;
  
  int max_itteration = ceil(log2(float(length)));
  float distance_start =  sqrt(pow(xy_positions[startpoint][1] - xy_positions[startpoint + length - 1][1],2) + pow(xy_positions[startpoint][2] - xy_positions[startpoint + length - 1][2],2));
  int bestpoint = startpoint;
  float bestdist = distance_start;
  int *best_point = &bestpoint;
  float *best_dist = &bestdist;
  int j = floor(float(length)/(2));
  search_longest(startpoint, startpoint+length-1,startpoint+j-1, length, distance_start, 1, max_itteration, best_point, best_dist);
  if((*best_dist) > distance_start*polygon_tolerance)
  { 
    if(polygon[*best_point] == 0)
    {   
      polygon[*best_point] = 1;
      polygon_size[shape_nr]++;
      extract_corners(startpoint,*best_point,*best_point-startpoint+1,shape_nr);
      extract_corners(*best_point,endpoint,endpoint-*best_point+1,shape_nr);
    }
  }
}

/* Searches for a point that would maximize the distance between the start and end point*/ 
void tracking_lidar::search_longest(int startpoint,int end_point, int current_point, int length, float distance_S, int itteration, int max_itteration, int *best_point, float *best_dist)
{

  if(length < 1 || (startpoint+length)>799)
    return;
  if(current_point < startpoint || current_point > end_point)
    return; 




  float distance_1, distance_2, distance_total;
  
  int j;
  itteration++;
  distance_1 =  sqrt(pow(xy_positions[startpoint][1] - xy_positions[current_point][1],2) + pow(xy_positions[startpoint][2] - xy_positions[current_point][2],2));
  distance_2 =  sqrt(pow(xy_positions[current_point][1] - xy_positions[end_point][1],2) + pow(xy_positions[current_point][2] - xy_positions[end_point][2],2));
  distance_total = distance_1 + distance_2;
  if(distance_total > *best_dist){
    *best_point = current_point;
    *best_dist = distance_total;
  }

  if(itteration < max_itteration)
    if(distance_total > distance_S*0.8)
    {
      if(distance_total < distance_S)
        distance_total = distance_S;

      j = floor(float(length)/pow(2,itteration));
      if(j > 0)
      {
        search_longest(startpoint,end_point, current_point+j, length, distance_total, itteration, max_itteration, best_point, best_dist);
        search_longest(startpoint,end_point, current_point-1, length, distance_total, itteration, max_itteration, best_point, best_dist);
      }
    }
}

/* Collects information polygon attributes for a simularity comparison*/
void tracking_lidar::polygon_attribute_extraction()
{
  int i,j;
  geometry_msgs::Point32 point1,point2,point3;
  float length1,length2,length3;
  for(i=0; i < MAX_OBJECTS; i++){
    object_attributes_list[i].longest_size = 0;
    object_attributes_list[i].sides_amount = 0;
    object_attributes_list[i].estimated_x = 0;
    object_attributes_list[i].estimated_y = 0;
    object_attributes_list[i].average_angle = 0;
    if(polygon_size[i] > 0){


      /* Get angle average, length average*/
      for(j=0;j < polygon_size[i]-1;j++)
      {
        point1 = shapes[i].points[j];
        point2 = shapes[i].points[j+1];

        length1 = sqrt(pow(point1.x - point2.x,2) + pow(point1.y - point2.y,2));
        if(length1 >object_attributes_list[i].longest_size)
         object_attributes_list[i].longest_size = length1;

        object_attributes_list[i].sides_amount++;

        object_attributes_list[i].estimated_x += point1.x;
        object_attributes_list[i].estimated_y += point1.y;

        if(polygon_size[i]-j > 2)
        {
          point3 = shapes[i].points[j+2];
          length2 = sqrt(pow(point3.x - point2.x,2) + pow(point3.y - point2.y,2));
          length3 = sqrt(pow(point3.x - point1.x,2) + pow(point3.y - point1.y,2));


          object_attributes_list[i].average_angle += acos((pow(length1,2) + pow(length2,2) - pow(length3,2))/(2*length1*length2));

        }

      }
      object_attributes_list[i].estimated_x += point2.x;
      object_attributes_list[i].estimated_y += point2.y;

      object_attributes_list[i].estimated_x /= polygon_size[i];
      object_attributes_list[i].estimated_y /= polygon_size[i];
      if(polygon_size[i]-2 > 0)
        object_attributes_list[i].average_angle /= polygon_size[i]-2;

      ROS_INFO("Polygon: Sides: %d, Longest Side %f, Average angle %f, x %f, y %f",object_attributes_list[i].sides_amount,object_attributes_list[i].longest_size,object_attributes_list[i].average_angle,object_attributes_list[i].estimated_x ,object_attributes_list[i].estimated_y);

    }
  }
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