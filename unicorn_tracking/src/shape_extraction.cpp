#include "shape_extraction.hpp"



shape_extraction::shape_extraction()
{
	 min_size_cluster = &min_size_cluster_std;
	 max_dist_laser = &max_dist_laser_std;
	 lambda = &lambda_std;
	 static_remove_dist = &static_remove_dist_std;
	 static_remove_ratio = &static_remove_ratio_std;
	 polygon_min_points = &polygon_min_points_std;
	 polygon_tolerance = &polygon_tolerance_std;
}


void shape_extraction::shape_extraction_setvar(float *lambda_p,float *max_dist_laser_p,int *static_remove_dist_p, float* static_remove_ratio_p,int* min_size_cluster_p, float *polygon_tolerance_p, int *polygon_min_points_p)
{
  // Roscore
		// Adaptive breakpoint
		lambda = lambda_p;
		max_dist_laser = max_dist_laser_p;
		// Filter
		static_remove_dist = static_remove_dist_p;
    static_remove_ratio = static_remove_ratio_p;
		min_size_cluster = min_size_cluster_p;
		// Polygon extraction
		polygon_tolerance = polygon_tolerance_p;
		polygon_min_points = polygon_min_points_p;
  // Point to variables

}

/* Cluster data with adaptive break point detection */
void shape_extraction::adaptive_break_point(const sensor_msgs::LaserScan& scan)
{
  int i,j = 0;
  float range, angle, r,p,dmax, x_t, y_t, x_previous = 0, y_previous = 0;
  //double roll,pitch,yaw,x,y,z;

  geometry_msgs::Point point;
  cluster cluster;
  cluster_list.clear();
  
  int max_itterations = round((scan.angle_max - scan.angle_min)/scan.angle_increment);
  
  for(i = 0; i < max_itterations+1; i++)
  {
    //polygon_point_list.push_back(clustered_point);
    angle = float(i)*scan.angle_increment+scan.angle_min;
    range = scan.ranges[i];

    // Transfor to Cartesian Coordinates
    point.x = range*cos(angle);
    point.y  = range*sin(angle);
    point.z = 0;

    if(i == 0)
    {
      cluster.cluster.clear();
      cluster.cluster.push_back(point);

      cluster.max_range = range;
      cluster.min_range = range;
    }else
    {
      r = sqrt(pow(x_previous,2) + pow(y_previous,2));
      p = sqrt(pow(x_previous - point.x,2) + pow(y_previous - point.y,2));
      dmax = r*sin(scan.angle_increment)/sin(*lambda + scan.angle_increment) + 3*scan.angle_increment;
      if(dmax < p && cluster.cluster.size()>0)
      {
        cluster.clusterNr = j;

        //std::cout << "Clsuters:" << cluster.clusterNr << " Cluster max range" << cluster.max_range << " Cluster min range" << cluster.min_range << " Cluster size" << cluster.cluster.size() <<  std::endl;
        cluster_list.push_back(cluster);
        cluster.cluster.clear();
        j++;
        cluster.max_range = range;
        cluster.min_range = range;
      }
      if(*max_dist_laser > range)
      {
      cluster.cluster.push_back(point);

      if(cluster.max_range < range)
        cluster.max_range = range;
      if(cluster.min_range > range)
        cluster.min_range = range;
      }
    }
    x_previous = point.x;
    y_previous = point.y;
  }
}



/* Filters static map from scan, requires reliable odometry and static map*/
void shape_extraction::static_map_filter(const nav_msgs::OccupancyGrid& map,const geometry_msgs::TransformStamped BaseFrame2Odomframe,const geometry_msgs::TransformStamped BaseLaser2BaseFrame,const geometry_msgs::TransformStamped odom2map)
{

  int i,j,m,n,ratio;
  int x_map,y_map;

  for(i=cluster_list.size()-1; i >= 0; i--)
  {
   
    ratio = 0;
    for(j=0; j < cluster_list[i].cluster.size(); j++)
    {
      if(cluster_list[i].clusterNr != -1)
      {
      geometry_msgs::Point point;
      point.x = cluster_list[i].cluster[j].x;
      point.y = cluster_list[i].cluster[j].y;
      point.z = cluster_list[i].cluster[j].z;
      tf2::doTransform(point,point,BaseLaser2BaseFrame);
      tf2::doTransform(point,point,BaseFrame2Odomframe);
      tf2::doTransform(point,point,odom2map);
      // Get MAP COORDINATES
      x_map = round((int)((float)point.x/(map.info.resolution))) + ((int)(map.info.width)/2);
      y_map = round((int)((float)point.y/(map.info.resolution))) + ((int)(map.info.height)/2);
      // Check Surrounding MAp positions 
      for(m=-*static_remove_dist; m <= *static_remove_dist;m++)
        for(n=-*static_remove_dist; n <= *static_remove_dist;n++){
          if(((x_map+m) < (int)map.info.height && (y_map+n) < (int)map.info.width)){
            if((0 < (x_map+m)) && (0 < (y_map+n)))
            {
              if(map.data[(x_map+m) + (y_map+n)*map.info.width] > 0){
                //cluster_list[i].clusterNr = -1;
                ratio++;
                m = *static_remove_dist+1;
                n = *static_remove_dist+1;
              }
            }
          }
        }
      }
    }
    //ROS_INFO("Number of clusters: %d - Points in cluster %d - Ratio found %f - Ratio needed %f", (int)cluster_list.size(),(int)cluster_list[i].cluster.size(), (ratio/(float)cluster_list[i].cluster.size()), *static_remove_ratio);
    if((ratio/(float)cluster_list[i].cluster.size()) > *static_remove_ratio)
    {
      //ROS_INFO("Clusters %d removed due to static filter", i);
      //cluster_list[i].cluster.clear();
      cluster_list.erase(cluster_list.begin() + i);
    }
  }
}

/* Transform objects to Global coordinates*/
void shape_extraction::transformObjects(const nav_msgs::Odometry& odometryData,const geometry_msgs::TransformStamped BaseLaser2BaseFrame)
{
  int i,j;
  for(i=cluster_list.size()-1; i >= 0; i--)
  {
    for(j=0; j < cluster_list[i].cluster.size(); j++)
    {
       cluster_list[i].cluster[j] = transform_point(cluster_list[i].cluster[j], odometryData,BaseLaser2BaseFrame);
    }
  }
}

/* Extracts polygon shapes from lidar clusters*/
void shape_extraction::polygon_extraction(){
  int i,j, current_cluster = 0, m = 0,n,l;
  int start_point = 0, end_point;
  geometry_msgs::Point32 point;
  object_attributes object;
 
  for(i=0; i < object_attributes_list.size(); i++)
    object_attributes_list[i].polygon.points.clear();

  if(object_attributes_list.size())
    object_attributes_list.clear();

  for(i=0; i < cluster_list.size(); i++)
  {
    std::vector<int> vextor(cluster_list[i].cluster.size(),0);
    
    if(cluster_list[i].clusterNr != -1)
      if(cluster_list[i].cluster.size() > *min_size_cluster){
        vextor[0] = 1;
        vextor[cluster_list[i].cluster.size()-1] = 1;
        extract_corners(0,cluster_list[i].cluster.size()-1, cluster_list[i].cluster.size(),i, &vextor);

        
        for(j=0; j < cluster_list[i].cluster.size(); j++)
        {
          if(vextor[j])
          {
            point.x = cluster_list[i].cluster[j].x;
            point.y = cluster_list[i].cluster[j].y;
            point.z = cluster_list[i].cluster[j].z;
            object.polygon.points.push_back(point);
          }
        }
        if(object.polygon.points.size() > 0) 
        {
          object.cluster = cluster_list[i].cluster;
          object_attributes_list.push_back(object);

          //std::cout << "Cluster nr " << i << " Clsuters size:" << object.cluster.size() << " Size Polygon" << object.polygon.points.size() << std::endl;
          object.polygon.points.clear();
        }
      }
  }
}

/*  Finds all corners between two points that satisfy a threshold*/
void shape_extraction::extract_corners(int startpoint,int endpoint, int length,int clusterNr, std::vector<int> *zeroVector)
{
  if(length < *polygon_min_points)
    return;
  
  int max_itteration = ceil(log2(float(length)));
  float distance_start =  sqrt(pow(cluster_list[clusterNr].cluster[startpoint].x - cluster_list[clusterNr].cluster[endpoint].x,2) + pow(cluster_list[clusterNr].cluster[startpoint].y - cluster_list[clusterNr].cluster[endpoint].y,2));
  int bestpoint = startpoint;
  float bestdist = distance_start;
  int *best_point = &bestpoint;
  float *best_dist = &bestdist;
  int j = floor(float(length)/(2));
  search_longest(startpoint, startpoint+length-1,startpoint+j-1, length, distance_start, 1, max_itteration, best_point, best_dist, clusterNr);
  if((*best_dist) > distance_start*(*polygon_tolerance))
  { 
    if((*zeroVector)[*best_point] == 0)
    {   
      (*zeroVector)[*best_point] = 1;
      extract_corners(startpoint,*best_point,*best_point-startpoint+1,clusterNr,zeroVector);
      extract_corners(*best_point,endpoint,endpoint-*best_point+1,clusterNr,zeroVector);
    }
  }
}

/* Searches for a point that would maximize the distance between the start and end point*/ 
void shape_extraction::search_longest(int startpoint,int end_point, int current_point, int length, float distance_S, int itteration, int max_itteration, int *best_point, float *best_dist, int clusterNr)
{
  if(length < 1 || (startpoint+length)>799)
    return;
  if(current_point < startpoint || current_point > end_point)
    return; 

  float distance_1, distance_2, distance_total;
  int j;
  
  itteration++;
  distance_1 =  sqrt(pow(cluster_list[clusterNr].cluster[startpoint].x - cluster_list[clusterNr].cluster[current_point].x,2) + pow(cluster_list[clusterNr].cluster[startpoint].y - cluster_list[clusterNr].cluster[current_point].y,2));
  distance_2 =  sqrt(pow(cluster_list[clusterNr].cluster[current_point].x - cluster_list[clusterNr].cluster[end_point].x,2) + pow(cluster_list[clusterNr].cluster[current_point].y - cluster_list[clusterNr].cluster[end_point].y,2));
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
        search_longest(startpoint,end_point, current_point+j, length, distance_total, itteration, max_itteration, best_point, best_dist,clusterNr);
        search_longest(startpoint,end_point, current_point-1, length, distance_total, itteration, max_itteration, best_point, best_dist,clusterNr);
      }
    }
}

/* Collects information polygon attributes for a simularity comparison*/
void shape_extraction::polygon_attribute_extraction()
{
  int i,j, m = 0;
  geometry_msgs::Point32 point1,point2,point3;
  float length1,length2,length3, angle,lowest_angle;
  for(i=0; i < object_attributes_list.size(); i++){
    object_attributes_list[i].length = -1;
    object_attributes_list[i].width = -1;
    object_attributes_list[i].sides_amount = 0;
    object_attributes_list[i].position.x = 0;
    object_attributes_list[i].position.y = 0;
    object_attributes_list[i].position.z = 0;
    object_attributes_list[i].average_angle = 0;
    if(object_attributes_list[i].polygon.points.size() >= 1){
      /* Get angle average, length average*/
      for(j=0;j < object_attributes_list[i].polygon.points.size()-1;j++)
      {
        point1 = object_attributes_list[i].polygon.points[j];
        point2 = object_attributes_list[i].polygon.points[j+1];
        length1 = sqrt(pow(point1.x - point2.x,2) + pow(point1.y - point2.y,2));
        if(length1 >object_attributes_list[i].length)
        {
          object_attributes_list[i].width = object_attributes_list[i].length;
          object_attributes_list[i].length = length1;
        }
         

        object_attributes_list[i].sides_amount++;
        object_attributes_list[i].position.x += point1.x;
        object_attributes_list[i].position.y += point1.y;
        if(object_attributes_list[i].polygon.points.size()-j > 2)
        {
          point3 = object_attributes_list[i].polygon.points[j+2];
          length2 = sqrt(pow(point3.x - point2.x,2) + pow(point3.y - point2.y,2));
          length3 = sqrt(pow(point3.x - point1.x,2) + pow(point3.y - point1.y,2));
          angle = acos((pow(length1,2) + pow(length2,2) - pow(length3,2))/(2*length1*length2));
          object_attributes_list[i].average_angle += angle;
        }
      }
      object_attributes_list[i].position.x += point2.x;
      object_attributes_list[i].position.y += point2.y;

      object_attributes_list[i].position.x /= object_attributes_list[i].polygon.points.size();
      object_attributes_list[i].position.y /= object_attributes_list[i].polygon.points.size();
      
      if(object_attributes_list[i].polygon.points.size()-2 > 0)
        object_attributes_list[i].average_angle /= object_attributes_list[i].polygon.points.size()-2;

    }
    //std::cout << "Cluster nr:" << i << " estimated_x:" << object_attributes_list[i].estimated_y << "estimated_y:"<< object_attributes_list[i].estimated_y <<std::endl;
  }
}

geometry_msgs::Point shape_extraction::transform_point(geometry_msgs::Point position, const nav_msgs::Odometry& odometryData,geometry_msgs::TransformStamped BaseLaser2BaseFrame)
{
  double roll,pitch,yaw,x_t,y_t;
  tf::Quaternion q(odometryData.pose.pose.orientation.x,odometryData.pose.pose.orientation.y,odometryData.pose.pose.orientation.z,odometryData.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll,pitch,yaw);
  // To base
  tf2::doTransform(position,position,BaseLaser2BaseFrame);
  // transform with odometry
  x_t = position.x;
  y_t = position.y;
  position.x = x_t*cos(yaw) - y_t*sin(yaw);
  position.y = y_t*cos(yaw) + x_t*sin(yaw);
  
  position.x += odometryData.pose.pose.position.x;
  position.y += odometryData.pose.pose.position.y;
  position.z = 0;

  return position;
  // transform to map
  //tf2::doTransform(point,point,odom2map);
}
