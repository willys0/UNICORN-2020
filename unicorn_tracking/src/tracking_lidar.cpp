#include "tracking_lidar.hpp"


static bool map_received = false;
static bool odom_received = false;
static bool scan_received = false;


/* Makes Variables reconfigurable */
void dynamicReconfigCallback(unicorn_tracking::TrackingConfig& config, uint32_t level, tracking_lidar *data) {
  data->lambda = config.lambda;
  data->max_dist_laser = config.max_dist_laser;
  data->static_filter = config.static_filter;
  data->static_remove_dist = config.static_remove_dist;
  data->polygon_tolerance = config.polygon_tolerance;
  data->polygon_min_points = config.polygon_min_points;
  data->min_twist_detection = config.min_twist_detection;
  data->sim_adj_dist = config.sim_adj_dist;
  data->sim_adj_angle = config.sim_adj_angle;
  data->sim_adj_side = config.sim_adj_side;
  data->sim_adj_xpos = config.sim_adj_xpos;
  data->sim_adj_ypos = config.sim_adj_ypos;
  data->max_similarty_deviation = config.max_similarty_deviation;
  data->min_size_cluster = config.min_size_cluster;

}



int main(int argc, char** argv){
  ros::init(argc, argv, "unicorn_tracking_node");

  ROS_INFO("Started Lidar Tracking node");
  tracking_lidar tracking_lidar_interface;
  
  dynamic_reconfigure::Server<unicorn_tracking::TrackingConfig> reconfig_server;
  reconfig_server.setCallback(boost::bind(&dynamicReconfigCallback, _1, _2, &tracking_lidar_interface));
  ros::Rate r(30.0);

  ros::spin();
}



/* Initiate varibles and callsbacks  */
tracking_lidar::tracking_lidar()
: n_("~")
{
  
  n_.param("lambda",lambda, 0.15f);
  n_.param("max_dist_laser",max_dist_laser, 10);

  n_.param("static_filter",static_filter, true);
  n_.param("Static_map_removal_tolerance",static_remove_dist, 4);

  n_.param("polygon_tolerance",polygon_tolerance, 1.04f);
  n_.param("polygon_side_min_points_required",polygon_min_points, 4);
  n_.param("Cluster_Lower_Limit",min_size_cluster, 5);
  

  n_.param("min_twist_detection",min_twist_detection, 0.08f);

  n_.param("similarty_side_length_weight",sim_adj_dist, 1.0f);
  n_.param("similarty_side_angle_weigh",sim_adj_angle, 1.0f);
  n_.param("similarty_side_amount_weight",sim_adj_side, 3.0f);
  n_.param("similarty_side_xposition_weight",sim_adj_xpos, 5.0f);
  n_.param("similarty_side_yposition_weight",sim_adj_ypos, 5.0f);

  n_.param("max_similarty_deviation",max_similarty_deviation, 1.5f);
  n_.param("map_topic",mapframeid, std::string("map")); //std::string("rear_laser")
  n_.param("base_laser_frame",base_laser_frame, std::string("base_laser")); 
  n_.param("base_frame",base_frame, std::string("chassis_link")); 
  n_.param("odomframeid",odomframeid, std::string("odom_chassis")); 

  
 

  //odometry_sub_ = n_.subscribe("/odom", 10, &tracking_lidar::odomCallback, this); /move_base/TebLocalPlannerROS/obstacles
  //scan_sub_ = n_.subscribe("/scan", 10, &tracking_lidar::scanCallback, this);
  odometry_sub_ = n_.subscribe("/odometry/filtered", 10, &tracking_lidar::odomCallback, this);
  scan_sub_ = n_.subscribe("/frontLidar/scan", 10, &tracking_lidar::scanCallback, this);
  // object_pub_ = n_.advertise<costmap_converter::ObstacleArrayMsg>("obstacles",10,false);
  object_pub_ = n_.advertise<costmap_converter::ObstacleArrayMsg>("/move_base/TebLocalPlannerROS/obstacles",10,false);
  marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("markerArray",10,false);
  marker_Arrow_pub_ = n_.advertise<visualization_msgs::MarkerArray>("markerArrowArray",10,false);

  tf2_ros::TransformListener tf2_listener(tf_buffer);
  Lidar2base = tf_buffer.lookupTransform(base_frame, base_laser_frame, ros::Time(0), ros::Duration(100.0) );
 

  //tf2_geometry_msgs

  if(static_filter)
  {
    map_sub_ = n_.subscribe("/map", 10, &tracking_lidar::mapCallback, this);
  }else
    map_received = true;


  seq = 0;

  memset(polygon_size, 0, MAX_OBJECTS*sizeof(polygon_size[0])); 

  initiate_Trackers();
}

/* Publishes data to topics */
void tracking_lidar::object_publisher()
{
  costmap_converter::ObstacleMsg object;
  costmap_converter::ObstacleArrayMsg object_array;
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markerArray, markerDArray;
  geometry_msgs::Point point;
  tf2::Quaternion Quad;

  object.header.frame_id = mapframeid;
  seq++;
  object.header.seq = seq;
  object.header.stamp = scan_data_.header.stamp;

  object.orientation.x = 0;
  object.orientation.y = 0;
  object.orientation.z = 0;
  object.orientation.w = 1;

  object.velocities.twist.linear.x = 0;
  object.velocities.twist.linear.y = 0;
  object.velocities.twist.linear.z = 0;

  object.velocities.twist.angular.x = 0;
  object.velocities.twist.angular.y = 0;
  object.velocities.twist.angular.z = 0;

  marker.header = object.header;
  marker.ns = "Tracking";
  
  marker.action = 0;
  marker.lifetime.nsec = 0;
  marker.lifetime.sec = 1;

  marker.scale.x = 0.15;
  marker.scale.y = 0.15;
  marker.scale.z = 0.15;

  marker.color.a = 1;
  marker.color.b = 1;
  marker.color.g = 0;
  marker.color.r = 0;

  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;

  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;

  object_array.obstacles.clear();
  object_array.header = object.header;
  int i,m;
  for(i=0; i < MAXTRACKS; i++)
  {
    if(trackers[i].age > CONFIRMED_TRACK){
      float min_length_arrow = sqrt(pow(trackers[i].tracker.x_hat(1),2) + pow(trackers[i].tracker.x_hat(3),2));
      marker.type = 5;

      object.polygon.points.clear();
      object.polygon = trackers[i].points;

      if(min_length_arrow > min_twist_detection)
      {
        object.velocities.twist.linear.x = trackers[i].tracker.x_hat(1);
        object.velocities.twist.linear.y = trackers[i].tracker.x_hat(3);
      }else{
        object.velocities.twist.linear.x = 0;
        object.velocities.twist.linear.y = 0;
      }

      Quad.setRPY(0,0,atan2(object.velocities.twist.linear.y,object.velocities.twist.linear.x));

      object_array.obstacles.push_back(object);

      marker.color.a = trackers[i].color[0]*(TRACKER_LIFE - (trackers[i].last_seen+1))/TRACKER_LIFE;
      marker.color.b = trackers[i].color[1];
      marker.color.g = trackers[i].color[2];
      marker.color.r = trackers[i].color[3];
      marker.pose.position.x = trackers[i].tracker.x_hat(0);
      marker.pose.position.y = trackers[i].tracker.x_hat(2);
      
      marker.id = i;


      marker.points.clear();
      for(m =0; m < trackers[i].sides_amount; m++)
      {
        point.x = trackers[i].points.points[m].x - trackers[i].tracker.x_hat(0);
        point.y = trackers[i].points.points[m].y - trackers[i].tracker.x_hat(2);
        point.z = trackers[i].points.points[m].z;
        marker.points.push_back(point);
        point.x = trackers[i].points.points[m+1].x - trackers[i].tracker.x_hat(0);
        point.y = trackers[i].points.points[m+1].y - trackers[i].tracker.x_hat(2);
        point.z = trackers[i].points.points[m+1].z;
        marker.points.push_back(point);
      }
      markerArray.markers.push_back(marker);
     
      if((min_length_arrow > min_twist_detection) && trackers[i].last_seen == 0)
      {
        marker.type = 0;
        marker.points.clear();
        point.x = 0;
        point.y = 0;
        point.z = 0;
        marker.points.push_back(point);
        point.x = trackers[i].tracker.x_hat(1)/min_length_arrow;
        point.y = trackers[i].tracker.x_hat(3)/min_length_arrow;
        point.z = 0;
        marker.points.push_back(point);
        markerDArray.markers.push_back(marker);
      }
     
    }
  }
  object_pub_.publish(object_array);
  marker_pub_.publish(markerArray);
  marker_Arrow_pub_.publish(markerDArray);
  markerArray.markers.clear();
  object_array.obstacles.clear();
  
}

/* Odometry Call back */
void tracking_lidar::odomCallback(const nav_msgs::Odometry& odometry)
{
    odometry_data_ = odometry;
    
    tf::Quaternion q(odometry.pose.pose.orientation.x,odometry.pose.pose.orientation.y,odometry.pose.pose.orientation.z,odometry.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    /* Get Euler angles */
    m.getRPY(roll,pitch,yaw);
    //yaw = -yaw; 
    x = odometry.pose.pose.position.x;
    y = odometry.pose.pose.position.y;
    z = odometry.pose.pose.position.z;
    odom_received = true;
}

/* Get map  */
void tracking_lidar::mapCallback(const nav_msgs::OccupancyGrid& map)
{
    map_data_ = map;
    if(!map_received){
      map_received = true;
      ROS_INFO("Map Received");
    }
    
    mapx = map_data_.info.height;
    mapy = map_data_.info.width;
}

/* Scan callback, each frame is one scan*/
void tracking_lidar::scanCallback(const sensor_msgs::LaserScan& scan)
{
    scan_data_ = scan;
    scan_received = true;

    

    
    if(map_received && odom_received && scan_received){
      adaptive_breaK_point();
      if(static_filter)
        static_map_filter();

      polygon_extraction();
      polygon_attribute_extraction();

      
      estimate_new_position();
      association();
      update_position();
      object_publisher();
      scan_received = false; 
    }/*
    else if(!map_received){
      ROS_INFO("Waiting for map, no map recieved!");
    }else if(!odom_received){
      ROS_INFO("Waiting odometry, no odometry recieved!");
    }*/
}

/* Associates objects with trackers*/
void tracking_lidar::association()
{
  int i,j,m = 0;
  double dt,time = scan_data_.header.stamp.sec + scan_data_.header.stamp.nsec*pow(10, -9);
  float s1,s2,s3,s4,s45,s5,s55,similarity,sim_adj,x_dot,y_dot;

  vector<vector<double>> object_match_ratio(MAX_OBJECTS,vector<double>(MAXTRACKS));


  /* Sets start values */
  memset(object_match,0,sizeof(object_match[0])*MAX_OBJECTS);
  //memset(object_match_ratio,0,sizeof(object_match_ratio[0][0])*MAX_OBJECTS*MAXTRACKS);

  /**/
  for(i=0; i<MAX_OBJECTS; i++)
    for(j=0; j<MAXTRACKS; j++)
        object_match_ratio[i][j] = 100;

  /* Check object against current trackers and gets similarity index*/
  for(i=0; i<MAXTRACKS; i++)
  {
    if(trackers[i].age > 0){
      trackers[i].age++;
      trackers[i].last_seen++;

      dt = time - trackers[i].time;  
      for(j=0; j<MAX_OBJECTS; j++)
      {
        if(polygon_size[j] >= 1){
          s1 = sim_adj_angle*abs((trackers[i].average_angle) - (object_attributes_list[j].average_angle));
          s2 = sim_adj_dist*abs((trackers[i].longest_size) - (object_attributes_list[j].longest_size));
          s3 = sim_adj_side*abs((trackers[i].sides_amount) - (object_attributes_list[j].sides_amount));  
          s4 = sim_adj_xpos*abs((trackers[i].tracker.x_hat(0)) - (object_attributes_list[j].estimated_x));
          s5 = sim_adj_ypos*abs((trackers[i].tracker.x_hat(2)) - (object_attributes_list[j].estimated_y));
          /**/
          if(trackers[i].last_seen > 0)
          {
            
          }

          if(s1 > s2)
          {
            similarity = s2;
          }else{
            similarity = s1;
          }
          if(isnan(s4 + s5))
          {
            s4 = 5;
            s5 = 5;
          }
          similarity += s3 + s4 + s5;
          similarity /= (sim_adj_dist + sim_adj_angle + sim_adj_side + sim_adj_xpos + sim_adj_ypos);
          if(trackers[i].age > CONFIRMED_TRACK)
            similarity *= 0.9;
          if(!isnan(similarity))
            object_match_ratio[i][j] = double(similarity); 
        }
      }  
    }
  }

  /* Hungarian algorithm to find the best match for the trackers */
  HungarianAlgorithm HungAlgo;
  vector<int> assignment;
  double cost = HungAlgo.Solve(object_match_ratio, assignment);

  
  /* Associates trackers according to cost*/
  for (m = 0; m < MAXTRACKS; m++)
  {
    if(object_match_ratio[m][assignment[m]] < max_similarty_deviation)
    {
      dt = time - trackers[m].time;
      object_match[assignment[m]] = 1; 
      trackers[m].last_seen = 0; 
      j = assignment[m];

      trackers[m].average_angle = object_attributes_list[j].average_angle;
      trackers[m].longest_size = object_attributes_list[j].longest_size;
      trackers[m].sides_amount = object_attributes_list[j].sides_amount;

      x_dot = (trackers[m].tracker.x_hat(0)-object_attributes_list[j].estimated_x)/float(dt);
      y_dot = (trackers[m].tracker.x_hat(2)-object_attributes_list[j].estimated_y)/float(dt);
      if(isnan(x_dot))
        x_dot = 0;
      if(isnan(y_dot))
        y_dot = 0;

      trackers[m].tracker.y << object_attributes_list[j].estimated_x, x_dot, object_attributes_list[j].estimated_y,y_dot; 
      trackers[m].points.points.clear();
      trackers[m].points = shapes[j];
      trackers[m].time = time;
    }else{
      trackers[m].last_seen++;
    }
      
  }
		 
  /*  Remove old trackers */
  for(i=0; i<MAXTRACKS; i++){
    if(trackers[i].age > 0){
      if((trackers[i].last_seen > TRACKER_LIFE) || (trackers[i].age < CONFIRMED_TRACK && trackers[i].last_seen > 1))
      {
        trackers[i].tracker.initialized = false;
        trackers[i].age = 0;
      }
    }
  }

   
  /* Adds new trackers */
  m = 0;
  for(j=0; j<MAX_OBJECTS; j++)
  {
    if(trackers[j].age > 0)
      m++;
    if(polygon_size[j] >= 1 && object_match[j] == 0)
    {
      for(i=0; i<MAXTRACKS; i++)
      {    
        if(trackers[i].age == 0)
        {
          trackers[i].age = 1;
          trackers[i].average_angle = object_attributes_list[j].average_angle;
          trackers[i].longest_size = object_attributes_list[j].longest_size;
          trackers[i].sides_amount = object_attributes_list[j].sides_amount;
          trackers[i].time = time;
          trackers[i].last_seen = 0;
          trackers[i].tracker.init();
          trackers[i].tracker.x_hat << object_attributes_list[j].estimated_x, 0,object_attributes_list[j].estimated_y, 0;
          trackers[i].tracker.y << object_attributes_list[j].estimated_x, 0, object_attributes_list[j].estimated_y, 0;
          trackers[i].color[0] = 1;
          trackers[i].color[1] = float(rand() % 80 + 20)/100;
          trackers[i].color[2] = float(rand() % 80 + 20)/100;
          trackers[i].color[3] = float(rand() % 80 + 20)/100;
          trackers[i].points.points.clear();
          trackers[i].points = shapes[j];
          break;
        }
      }
    }
  }
  ROS_INFO("Number of trackers %d",m);
}

/* Initiates all trackers */
void tracking_lidar::initiate_Trackers()
{
  double dt = 1.0/30;

  // Construct the filter
  KalmanFilter kf;
  kf.A << 1, dt, 0, 0, 0, 1, 0, 0, 0, 0, 1, dt, 0, 0, 0, 1;
  kf.C << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; 
  kf.P0 << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0, 0, 0, 0, 0.05, 0.05, 0, 0, 0.05, 0.05;
  // Tunable noise matrices
  kf.Q << 0.05, 0.05, 0, 0, 0.05, 0.05, 0, 0, 0, 0, 0.05, 0.05, 0, 0, 0.05, 0.05;
  kf.R << 0.05, 0.05, 0, 0, 0, 0.2, 0, 0, 0, 0, 0.05, 0.05, 0, 0, 0, 0.2; 
  
  int i;
  for(i=0; i<MAXTRACKS; i++)
  {
      trackers[i].tracker = kf;
      trackers[i].age = 0;
      trackers[i].average_angle = 0;
      trackers[i].confirmed = 0;
      trackers[i].longest_size = 0;
      trackers[i].sides_amount = 0;
      trackers[i].time = 0;
  }
}

/* Updates A matrix depending on the time differnce and estimates new position*/
void tracking_lidar::estimate_new_position()
{
  int i;
  double dt, time = scan_data_.header.stamp.sec + scan_data_.header.stamp.nsec*pow(10, -9);
  for(i=0; i<MAXTRACKS; i++)
  {
    if(trackers[i].age > 0)
    {
      dt = time - trackers[i].time;
      trackers[i].tracker.A << 1, dt, 0, 0, 0, 1, 0, 0, 0, 0, 1, dt, 0, 0, 0, 1;
      trackers[i].tracker.predict();
    }
  }
}

/* Update position with new position estimate*/
void tracking_lidar::update_position(){
  int i;
  for(i=0; i<MAXTRACKS; i++)
  {
    if(trackers[i].age > 0 && trackers[i].last_seen == 0)
    {
      trackers[i].tracker.update(trackers[i].tracker.y);
      if(isnan(trackers[i].tracker.x_hat(0)+trackers[i].tracker.x_hat(1)+trackers[i].tracker.x_hat(2)+trackers[i].tracker.x_hat(3)))
        trackers[i].age = 0;
    }
  }
}

/* Clusters data into groups */
void tracking_lidar::adaptive_breaK_point()
{
  int i,j = 0;
  float current_angle,r,p,dmax;
  geometry_msgs::Point point;
  int max_itterations = round((scan_data_.angle_max - scan_data_.angle_min)/scan_data_.angle_increment);
  odom2map = tf_buffer.lookupTransform(mapframeid, base_laser_frame, ros::Time(0), ros::Duration(100.0) );
  for(i = 0; i < max_itterations+1; i++)
  {
    current_angle = float(i)*scan_data_.angle_increment+scan_data_.angle_min;
    // Calculate xy position
    //xy_positions[i][1] = (scan_data_.ranges[i])*(cos(current_angle)*cos(yaw) + sin(current_angle)*sin(yaw)) + x;
    //xy_positions[i][2] = (scan_data_.ranges[i])*(sin(current_angle)*cos(yaw) - cos(current_angle)*sin(yaw)) + y;

    point.x = (scan_data_.ranges[i])*(cos(current_angle)*cos(yaw) + sin(current_angle)*sin(yaw));
    point.y = (scan_data_.ranges[i])*(sin(current_angle)*cos(yaw) - cos(current_angle)*sin(yaw));
    point.z = 0;
    //tf2::doTransform(point,point,Lidar2base);
    //point.x  + x;
    //point.y  + y;
    tf2::doTransform(point,point,odom2map);
    xy_positions[i][1] = point.x;
    xy_positions[i][2] = point.y;
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
    /* Filters scans that over a set threshold*/
    if(scan_data_.ranges[i] > max_dist_laser)
      clusters[i] = -1;
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
  int i,j, current_cluster = clusters[0], m = 0,n,l;
  int start_point = 0, end_point;
  geometry_msgs::Point32 point;

  // Clear shape array
  for(i=0; i < MAX_OBJECTS; i++)
  {
    if(polygon_size[i] >= 1)
    {
      shapes[i].points.clear();
      polygon_size[i] = 0;
    }
  }

  for(i=1; i < 800; i++){
    polygon[i] = 0;
    if(clusters[i] != current_cluster){ 
      end_point = i-1;
      if(current_cluster != -1 && end_point-start_point > min_size_cluster){
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
            m++;
          }
      }
      current_cluster = clusters[i];
      start_point = i;
    }
  }
}

/*  Finds all corners between two points that satisfy a threshold*/
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
  int i,j, m = 0;
  geometry_msgs::Point32 point1,point2,point3;
  float length1,length2,length3, angle,lowest_angle;
  for(i=0; i < MAX_OBJECTS; i++){
    object_attributes_list[i].longest_size = 0;
    object_attributes_list[i].sides_amount = 0;
    object_attributes_list[i].estimated_x = 0;
    object_attributes_list[i].estimated_y = 0;
    object_attributes_list[i].average_angle = 0;
    lowest_angle = 10;
    if(polygon_size[i] >= 1){
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
          angle = acos((pow(length1,2) + pow(length2,2) - pow(length3,2))/(2*length1*length2));
          object_attributes_list[i].average_angle += angle;
          if(angle < lowest_angle)
          {
            lowest_angle = angle;
            m = j+1;
          }
        }
      }
      object_attributes_list[i].estimated_x += point2.x;
      object_attributes_list[i].estimated_y += point2.y;

      object_attributes_list[i].estimated_x /= polygon_size[i];
      object_attributes_list[i].estimated_y /= polygon_size[i];
      if(polygon_size[i]-2 > 0)
        object_attributes_list[i].average_angle /= polygon_size[i]-2;

      if(lowest_angle < 2)
      {
        object_attributes_list[i].estimated_x = shapes[i].points[m].x;
        object_attributes_list[i].estimated_y = shapes[i].points[m].y;
      }
    }
  }
}

