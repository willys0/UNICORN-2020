#include "tracking_lidar.hpp"



static bool map_received = false;
static bool odom_received = false;
static bool scan_received = false;


int main(int argc, char** argv){
  ros::init(argc, argv, "tracking_lidar");


  ROS_INFO("Started Lidar Tracking node");
  tracking_lidar tracking_lidar_interface;
  ros::Rate r(30.0);

/*
while(ros::ok)
  {


    ros::spinOnce();
    r.sleep();
    
    
    if(map_received && odom_received && scan_received){
      //ROS_INFO(" Running ");
      tracking_lidar_interface.association();
      tracking_lidar_interface.object_publisher();
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
  object_pub_ = n_.advertise<costmap_converter::ObstacleArrayMsg>("obstacles",10,false);
  marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("markerArray",10,false);
  n_.param("lambda",lambda, 0.15f);
  n_.param("Max_Laser_dist",max_dist_laser, 10);
  n_.param("Static_map_removal_tolerance",static_remove_dist, 4);
  n_.param("polygon_tolerance",polygon_tolerance, 1.04f);
  n_.param("polygon_min_points_required",polygon_min_points, 4);

  seq = 0;

  memset(polygon_size, 0, MAX_OBJECTS*sizeof(polygon_size[0])); 

  initiate_Trackers();


}



void tracking_lidar::object_publisher()
{
  costmap_converter::ObstacleMsg object;
  costmap_converter::ObstacleArrayMsg object_array;
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markerArray;
  geometry_msgs::Point point;
  
  std::string frameid ("/map");

  object.header.frame_id = frameid;
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
  marker.type = 5;
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
  marker.pose.position.z = 0.1;

  object_array.obstacles.clear();
  object_array.header = object.header;
  int i,m;
  for(i=0; i < MAXTRACKS; i++)
  {
    if(trackers[i].age > 0){

      //ROS_INFO("PRINTING tracker: %d",i);

      object.polygon.points.clear();
      object.polygon = trackers[i].points;
/**/
      marker.color.a = trackers[i].color[0]*(TRACKER_LIFE - (trackers[i].last_seen+1))/TRACKER_LIFE;
      marker.color.b = trackers[i].color[1];
      marker.color.g = trackers[i].color[2];
      marker.color.r = trackers[i].color[3];

      object_array.obstacles.push_back(object);
      


      marker.id = i;
      marker.points.clear();
      for(m =0; m < trackers[i].sides_amount; m++)
      {
      point.x = trackers[i].points.points[m].x;
      point.y = trackers[i].points.points[m].y;
      point.z = trackers[i].points.points[m].z;
      marker.points.push_back(point);
      point.x = trackers[i].points.points[m+1].x;
      point.y = trackers[i].points.points[m+1].y;
      point.z = trackers[i].points.points[m+1].z;
      marker.points.push_back(point);
      }
      markerArray.markers.push_back(marker);
     
    }
  }
  //ROS_INFO("Test pub");
  object_pub_.publish(object_array);
  marker_pub_.publish(markerArray);
  markerArray.markers.clear();
  object_array.obstacles.clear();
  
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
      //estimate_new_position();
      //update_tracker; 
      association();
      //update_position();
      
      // https://www.intechopen.com/books/kalman-filters-theory-for-advanced-applications/kalman-filter-for-moving-object-tracking-performance-analysis-and-filter-design good read

      //predict_postions
      object_publisher();
      scan_received = false;
      

    }
    
    // Do stuff here 
}

void tracking_lidar::association()
{
/* */
    //int numberOfTracks = multitracker.activeTracks;
    int i,j,m = 0;
    double dt,time = scan_data_.header.stamp.sec + scan_data_.header.stamp.nsec*pow(10, -9);

    memset(object_match,0,sizeof(object_match[0])*MAX_OBJECTS);

    Eigen::VectorXd estimatedPosition(3), outputposition(3); 
    vector<vector<double>> object_match_ratio(MAX_OBJECTS,vector<double>(MAXTRACKS));

    //object_match_ratio.fill(100);
  /**/
    for(i=0; i<MAX_OBJECTS; i++)
      for(j=0; j<MAXTRACKS; j++)
          object_match_ratio[i][j] = 100;
  

    float similarity = 0,similarity_best = 0;

    float s1,s2,s3,s4,s5;
    Eigen::VectorXd y{2},x{3};

    
    /**/
    // Check object against current trackers

    for(i=0; i<MAXTRACKS; i++)
    {
      if(trackers[i].age > 0){
        //ROS_INFO("Tracker %d last seen %d - pos %lf %lf ",i, trackers[i].last_seen ,trackers[i].tracker.x_hat(0),trackers[i].tracker.x_hat(1));
        m = -1;
        similarity_best = 0;
        /* */
        trackers[i].age++;
        trackers[i].last_seen++;
        dt = time - trackers[i].time;
        //trackers[i].tracker.A << 1, 0, 0, 0, 1, 0, 0, 0, 1;

        //ROS_INFO("Tract  after  ");
        //std::cout << trackers[i].tracker.x_hat << std::endl;
        //trackers[i].tracker.predict();
        //std::cout << trackers[i].tracker.x_hat_new << std::endl;
        
        // Check against all objects
        // add some kind of confirmation tracks with adjustment benefit
       
        for(j=0; j<MAX_OBJECTS; j++)
        {
          if(polygon_size[j] > 0){
            /**/
            similarity = abs((trackers[i].average_angle) - (object_attributes_list[j].average_angle));
            
            similarity += abs((trackers[i].longest_size) - (object_attributes_list[j].longest_size));
            similarity += abs((trackers[i].sides_amount) - (object_attributes_list[j].sides_amount)); 
            
            similarity += abs((trackers[i].tracker.x_hat_new(0)) - (object_attributes_list[j].estimated_x));
            similarity += abs((trackers[i].tracker.x_hat_new(2)) - (object_attributes_list[j].estimated_y));
            
            similarity /= 5;

            if(!isnan(similarity))
              object_match_ratio[i][j] = double(similarity); 
            //ROS_INFO("object match %f",similarity);
 
             

                
          }
        }  
        //y << object_attributes_list[j].estimated_x, object_attributes_list[j].estimated_y;
        //trackers[i].tracker.update(y);

      }
    }

    /* Hungarian algorithm to find the lowest cost combination of trackers */
    HungarianAlgorithm HungAlgo;
	  vector<int> assignment;
	  double cost = HungAlgo.Solve(object_match_ratio, assignment);

    /**/
    for(i=0; i<MAX_OBJECTS; i++)
    {
      for(j=0; j<MAXTRACKS; j++)
      {
        std::cout << std::setprecision(3) << object_match_ratio[i][j] << "    ";
      }
      std::cout << std::endl;
    }

	  for (m = 0; m < MAXTRACKS; m++)
    {
      //std::cout << m << "," << assignment[m] << "-" << object_match_ratio[m][assignment[m]] << "\t";
      if(object_match_ratio[m][assignment[m]] < 1.5)
      {
        ROS_INFO("Object %d identified wiuth tracker %d",assignment[m], m);
        object_match[assignment[m]] = 1; 
        trackers[m].last_seen = 0; // update last seen
        j = assignment[m];

        trackers[m].average_angle = object_attributes_list[j].average_angle;
        trackers[m].longest_size = object_attributes_list[j].longest_size;
        trackers[m].sides_amount = object_attributes_list[j].sides_amount;
        trackers[m].tracker.y << object_attributes_list[j].estimated_x, 0, object_attributes_list[j].estimated_y, 0; // should actually be y
        trackers[m].points.points.clear();
        trackers[m].points = shapes[j];
        trackers[m].time = time;

         

      }else{
        trackers[m].last_seen++;

      }
       
    }
		 
    // Remove old trackers
    for(i=0; i<MAXTRACKS; i++)
      if(trackers[i].age > 0)
        if(trackers[i].last_seen > TRACKER_LIFE)
        {
          ROS_INFO("Removing tracker %d",i);
          trackers[i].tracker.initialized = false;
          trackers[i].age = 0;
        }
   
   // Add new trackers 
   m = 0;
    for(j=0; j<MAX_OBJECTS; j++)
    {
      if(trackers[j].age > 0)
        m++;

      if(polygon_size[j] > 0 && object_match[j] == 0)
      {
        //ROS_INFO("tracker poly, object %d object %d",polygon_size[j], object_match[j]);
        for(i=0; i<MAXTRACKS; i++)
        {
          
          if(trackers[i].age == 0)
          {
            ROS_INFO("New Tracker %d for object %d",i,j);
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
    ROS_INFO("Number of active trackers %d",m);


}


void tracking_lidar::initiate_Trackers(){

/**/
  double dt = 1.0/30;

  // Construct the filter
  //KalmanFilter kf;
  KalmanFilter kf;
  kf.A << 1, dt, 0, 0, 0, 1, 0, 0, 0, 0, 1, dt, 0, 0, 0, 1;

  //kf.B << 0, 0, 0;
  kf.C << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; 
  kf.Q << 0.5, 0.5, 0, 0, 0.5, 0.5, 0, 0, 0, 0, 0.5, 0.5, 0, 0, 0.5, 0.5;
  kf.R << 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1; // Should probably depend on attributes of measure obstacle
  kf.P << 1, 0.5, 0, 0, 0.5, 1, 0, 0, 0, 0, 1, 0.5, 0, 0, 0.5, 1;
  

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
  
  //std::cout << trackers[1].tracker.A << std::endl;
  
}

void tracking_lidar::estimate_new_position(){
  int i;
  double dt, time = scan_data_.header.stamp.sec + scan_data_.header.stamp.nsec*pow(10, -9);
  
 
  for(i=0; i<MAXTRACKS; i++)
  {
    if(trackers[i].age > 0)
    {
      dt = time - trackers[i].time;
      trackers[i].tracker.A << 1, dt, 0, 0, 0, 1, 0, 0, 0, 0, 1, dt, 0, 0, 0, 1;
      trackers[i].tracker.predict();

      //std::cout << "Object " << i << " State predicted \n" << trackers[1].tracker.x_hat_new << std::endl;

    }

  }
}
void tracking_lidar::update_position(){
  int i;
  for(i=0; i<MAXTRACKS; i++)
  {
    if(trackers[i].age > 0)
    {
      trackers[i].tracker.update(trackers[i].tracker.y);

      //std::cout << "Object " << i << " State estimated \n" << trackers[1].tracker.x_hat << std::endl;

    }
  }

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

      //ROS_INFO("Polygon: Sides: %d, Longest Side %f, Average angle %f, x %f, y %f",object_attributes_list[i].sides_amount,object_attributes_list[i].longest_size,object_attributes_list[i].average_angle,object_attributes_list[i].estimated_x ,object_attributes_list[i].estimated_y);

    }
  }
}

