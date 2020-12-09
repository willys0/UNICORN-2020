#include "shape_extraction.hpp"



shape_extraction::shape_extraction()
{
  // Roscore

  // Point to variables

}

/* Clusters data into groups */
void shape_extraction::adaptive_breaK_point()
{
  int i,j = 0;
  float current_angle,r,p,dmax;
  double roll_map, pitch_map, yaw_map;
  geometry_msgs::Point point;
  int max_itterations = round((scan_data_.angle_max - scan_data_.angle_min)/scan_data_.angle_increment);

  odom2map = tf_buffer.lookupTransform(mapframeid, odomframeid, ros::Time(0),ros::Duration(5));
  
  for(i = 0; i < max_itterations+1; i++)
  {
    current_angle = float(i)*scan_data_.angle_increment+scan_data_.angle_min;

    // Calculate xy position
    point.x = (scan_data_.ranges[i])*(cos(current_angle)*cos(yaw) + sin(current_angle)*sin(yaw));
    point.y = (scan_data_.ranges[i])*(sin(current_angle)*cos(yaw) - cos(current_angle)*sin(yaw));
    point.z = 0;


    tf2::doTransform(point,point,Lidar2base);

    point.x += x;
    point.y += y;
    point.z = 0;


    // transform to map
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
void shape_extraction::static_map_filter()
{
  int i,m,n;
  uint32_t x_map,y_map;
  for(i=0; i < SCAN_SIZE;i++){
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
void shape_extraction::polygon_extraction(){
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


  object_attributes_list[m].points.points.clear();
  for(i=1; i < SCAN_SIZE; i++){
    polygon[i] = 0;
    point.x = xy_positions[i][1];
    point.y = xy_positions[i][2];
    point.z = 0;
    object_attributes_list[m].points.points.push_back(point);
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
            object_attributes_list[m].points.points.clear();
          }
      }
      current_cluster = clusters[i];
      start_point = i;
    }
  }
}

/*  Finds all corners between two points that satisfy a threshold*/
void shape_extraction::extract_corners(int startpoint,int endpoint, int length,int shape_nr)
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
void shape_extraction::search_longest(int startpoint,int end_point, int current_point, int length, float distance_S, int itteration, int max_itteration, int *best_point, float *best_dist)
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
void shape_extraction::polygon_attribute_extraction()
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
        }
      }
      object_attributes_list[i].estimated_x += point2.x;
      object_attributes_list[i].estimated_y += point2.y;

      object_attributes_list[i].estimated_x /= polygon_size[i];
      object_attributes_list[i].estimated_y /= polygon_size[i];
      if(polygon_size[i]-2 > 0)
        object_attributes_list[i].average_angle /= polygon_size[i]-2;

    }
  }
}