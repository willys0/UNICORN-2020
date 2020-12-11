#include "association.hpp"


association::association()
{


}

/* Associates objects with trackers*/
void association::associate(const std::vector<shape_extraction::object_attributes>& object_attributes_list,const nav_msgs::Odometry& odometryData,const geometry_msgs::TransformStamped BaseLaser2BaseFrame, ros::Time stamp)
{
  int i,j,m = 0;
  double dt,time = stamp.sec + stamp.nsec*pow(10, -9);
  float s1,s2,s3,s4,s5,s6,similarity,sim_adj,x_dot,y_dot, sum[3];

  vector<vector<double>> object_match_ratio(MAX_OBJECTS,vector<double>(MAX_OBJECTS));
  std::vector<int> object_match(MAX_OBJECTS,0);

  //std::cout << "vector:";
  //for(i=0; i<MAX_OBJECTS; i++)
  //    std::cout << object_match[i];
  //std::cout << "" << std::endl;
  estimate_new_position(time);

  /* Sets start values */
  //memset(object_match_ratio,0,sizeof(object_match_ratio[0][0])*MAX_OBJECTS*MAXTRACKS);

  /**/
  for(i=0; i<MAX_OBJECTS; i++)
    for(j=0; j<MAX_OBJECTS; j++)
        object_match_ratio[i][j] = 100;


  int objects_size = object_attributes_list.size();
  if(objects_size > MAX_OBJECTS)
    objects_size = MAX_OBJECTS;

  /* Check object against current trackers and gets similarity index*/  
  for(i=0; i<trackers.size(); i++)
  {
    //ROS_INFO("Tracker test %d",i);
    if(trackers[i].age > 0){
      trackers[i].age++;
      trackers[i].last_seen++;

      dt = time - trackers[i].time;  
      for(j=0; j<objects_size; j++)
      {
        if(object_attributes_list[j].polygon.points.size() >= 1){
          calculateVel(object_attributes_list[j], i,sum);
          s1 = sim_adj_angle*abs((trackers[i].average_angle) - (object_attributes_list[j].average_angle));
          s2 = sim_adj_dist*abs((trackers[i].longest_size) - (object_attributes_list[j].longest_size));
          s3 = sim_adj_side*abs((trackers[i].sides_amount) - (object_attributes_list[j].sides_amount));  

          geometry_msgs::Point point = transform_point(object_attributes_list[j].position, odometryData,BaseLaser2BaseFrame);
          s4 = sim_adj_xpos*abs((trackers[i].tracker.x_hat(0)) - (point.x));
          s5 = sim_adj_ypos*abs((trackers[i].tracker.x_hat(2)) - (point.y));


          //s6 = sim_adj_posdiff*abs(s6);
          s6 = sim_adj_posdiff;
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
          if(isnan(s6))
          {
            s6 = sim_adj_posdiff;
          }
          similarity += s3 + s4 + s5 + s6;
          similarity /= (sim_adj_dist + sim_adj_angle + sim_adj_side + sim_adj_xpos + sim_adj_ypos + sim_adj_posdiff);
          
          if(trackers[i].age > CONFIRMED_TRACK)
            similarity *= 0.9;
          if(!isnan(similarity))
            object_match_ratio[i][j] = double(similarity); 

          //ROS_INFO("Test _ sim %d - components %d %d %d %d %d ",similarity,s5,s4);
        }
      }  
    }
  }

  /* Hungarian algorithm to find the best match for the trackers */
  HungarianAlgorithm HungAlgo;
  vector<int> assignment;
  double cost = HungAlgo.Solve(object_match_ratio, assignment);


  /*
  for (m = 0; m < 100; m++)
    std::cout << "Tracker M:" << m << " Object:" << assignment[m] << " Cost" << object_match_ratio[m][assignment[m]] << std::endl;*/
  
  /* Associates trackers according to cost*//**/
  for (m = 0; m < trackers.size(); m++)
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
      

      
      calculateVel(object_attributes_list[j], m,sum);
      x_dot = sum[1];
      y_dot = sum[2];
      if(isnan(x_dot))
        x_dot = 0;
      if(isnan(y_dot))
        y_dot = 0;

      geometry_msgs::Point point = transform_point(object_attributes_list[j].position, odometryData,BaseLaser2BaseFrame);
      x_dot = (trackers[m].tracker.x_hat(0)-point.x)/float(dt);
      y_dot = (trackers[m].tracker.x_hat(2)-point.y)/float(dt);

      
      trackers[m].tracker.y << point.x, x_dot, point.y,y_dot; 
      trackers[m].points.points.clear();
     
      for(i = 0; i < object_attributes_list[j].polygon.points.size(); i++)
      {
        point.x = object_attributes_list[j].polygon.points[i].x;
        point.y = object_attributes_list[j].polygon.points[i].y;
        point.z = object_attributes_list[j].polygon.points[i].z;
        point = transform_point(point, odometryData, BaseLaser2BaseFrame);
        geometry_msgs::Point32 point32;
        point32.x = point.x;
        point32.y = point.y;
        point32.z = point.z;
        trackers[m].points.points.push_back(point32);
      }
      //trackers[m].points = object_attributes_list[j].polygon;
      trackers[m].time = time;
    }else{
      trackers[m].last_seen++;
    }
      
  }
	


  /*  Remove old trackers */
  j = 0;
  for(i=trackers.size()-1; i>=0; i--){
    j++;
    //ROS_INFO("Tracker %d, last seen %d age %d",i,trackers[i].last_seen, trackers[i].age);
    if(trackers[i].age > 0){
      if((trackers[i].last_seen > TRACKER_LIFE) || (trackers[i].age < CONFIRMED_TRACK && trackers[i].last_seen > 1))
      {

        trackers.erase(trackers.begin() + i);
       
        ROS_INFO("Tracker removed %d - trackers %d",i, (int)trackers.size());

      }
    }
  }

       
  /* Adds new trackers */
  for(j=0; j<object_attributes_list.size(); j++)
  {
    if(object_attributes_list[j].polygon.points.size() >= 1 && object_match[j] == 0)
    {
      if(trackers.size()<MAX_OBJECTS)
      {    
        ROS_INFO("Add tracker object %d",j);
        initiate_Trackers(object_attributes_list[j],time, odometryData,BaseLaser2BaseFrame);
      }
    }
  }
  //ROS_INFO("Number of trackers %d",(int)trackers.size());

  


  update_position();
}




/* Estimates the movment between an object a a tracker*/
void association::calculateVel(shape_extraction::object_attributes object, int trackernr,float *sum)
{
  int i,j;

  float x_t,y_t,x_o,y_o,distance;

  geometry_msgs::Polygon points;
  geometry_msgs::Point32 point1;
  
  point1.z = 10;
  point1.x = 0;
  point1.y = 0;
  sum[0] = 0;
  sum[1] = 0;
  sum[2] = 0;

  /* Checks minimum movent between each point recorded in the tracker and the repective object*/
  for(j=0;j < (int)(trackers[trackernr].points.points.size()); j++)
  {
    x_t = trackers[trackernr].points.points[j].x;
    y_t = trackers[trackernr].points.points[j].y;
    point1.z = 10;
    point1.x = 0;
    point1.y = 0;
    for(i=0;i < (int)(object.polygon.points.size());i++)
    {
      x_o = object.polygon.points[i].x;
      y_o = object.polygon.points[i].y;
      distance = sqrt(pow(x_o-x_t,2)+pow(y_o-y_t,2));
      // new positon
      if(distance < point1.z){
         point1.z = distance;
         point1.x = x_o-x_t;
         point1.y = y_o-y_t;
      }
    }
    if(point1.z < 10)
      points.points.push_back(point1);

  }
  j=1;
  for(i=0;i<(int)(points.points.size());i++)
    if(points.points[i].z < 10)
    {
      j++;
      sum[0] += points.points[i].z;
      sum[1] += points.points[i].x;
      sum[2] += points.points[i].y;
    }
  sum[0] /= j;
  sum[1] /= j;
  sum[2] /= j;
}


/* Initiates all trackers */
void association::initiate_Trackers(shape_extraction::object_attributes object,double time, const nav_msgs::Odometry& odometryData,geometry_msgs::TransformStamped BaseLaser2BaseFrame)
{
  double dt = 1.0/30;
  tracker_attributes new_trackers;

  // Construct the filter
  new_trackers.tracker.A << 1, dt, 0, 0, 0, 1, 0, 0, 0, 0, 1, dt, 0, 0, 0, 1;
  new_trackers.tracker.C << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1; 
  new_trackers.tracker.P0 << 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0, 0, 0, 0, 0.05, 0.05, 0, 0, 0.05, 0.05;
  // Tunable noise matrices
  new_trackers.tracker.Q << 0.05, 0.05, 0, 0, 0.05, 0.05, 0, 0, 0, 0, 0.05, 0.05, 0, 0, 0.05, 0.05;
  new_trackers.tracker.R << 0.05, 0.05, 0, 0, 0, 0.5, 0, 0, 0, 0, 0.05, 0.05, 0, 0, 0, 0.5; 

  new_trackers.age = 1;
  new_trackers.average_angle = object.average_angle;
  new_trackers.longest_size = object.longest_size;
  new_trackers.sides_amount = object.sides_amount;
  new_trackers.time = time;
  new_trackers.last_seen = 0;
  new_trackers.tracker.init();

  geometry_msgs::Point point = transform_point(object.position, odometryData,BaseLaser2BaseFrame);
  new_trackers.tracker.x_hat << point.x, 0,point.y, 0;
  new_trackers.tracker.y << point.x, 0, point.y, 0;

  // Set Random Colors
  new_trackers.color[0] = 1;
  new_trackers.color[1] = float(rand() % 80 + 20)/100;
  new_trackers.color[2] = float(rand() % 80 + 20)/100;
  new_trackers.color[3] = float(rand() % 80 + 20)/100;
  new_trackers.points.points.clear();

  int i;
  for(i = 0; i < object.polygon.points.size(); i++)
  {
    point.x = object.polygon.points[i].x;
    point.y = object.polygon.points[i].y;
    point.z = object.polygon.points[i].z;
    point = transform_point(point, odometryData, BaseLaser2BaseFrame);
    geometry_msgs::Point32 point32;
    point32.x = point.x;
    point32.y = point.y;
    point32.z = point.z;
    new_trackers.points.points.push_back(point32);
  }
//new_trackers.points = object.polygon;


  trackers.push_back(new_trackers);
}


/* Updates A matrix depending on the time differnce and estimates new position*/
void association::estimate_new_position(double time)
{
  int i;
  double dt;
  for(i=0; i<trackers.size(); i++)
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
void association::update_position(){
  int i;
  for(i=0; i<trackers.size(); i++)
  {
    if(trackers[i].age > 0 && trackers[i].last_seen == 0)
    {
      trackers[i].tracker.update(trackers[i].tracker.y);
      if(isnan(trackers[i].tracker.x_hat(0)+trackers[i].tracker.x_hat(1)+trackers[i].tracker.x_hat(2)+trackers[i].tracker.x_hat(3)))
        trackers[i].age = 0;
    }
  }
}


/*
void association::transform_polygon(geometry_msgs::Polygon polygon_in, geometry_msgs::Polygon polygon_out, const nav_msgs::Odometry& odometryData,geometry_msgs::TransformStamped BaseLaser2BaseFrame)
{
  int i;
 for(i = 0; i < polygon_in.points.size(); i++)
 {
   geometry_msgs::Point point = transform_point(polygon_in.points[i], odometryData, BaseLaser2BaseFrame);
   polygon_out.points.push_back(point);
 }

}*/

geometry_msgs::Point association::transform_point(geometry_msgs::Point position, const nav_msgs::Odometry& odometryData,geometry_msgs::TransformStamped BaseLaser2BaseFrame)
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
  position.x = x_t*cos(-yaw) + y_t*sin(-yaw);
  position.y = y_t*cos(-yaw) - x_t*sin(-yaw);
  
  position.x += odometryData.pose.pose.position.x;
  position.y += odometryData.pose.pose.position.y;
  position.z = 0;

  return position;
  // transform to map
  //tf2::doTransform(point,point,odom2map);

}


/*
using namespace gs;

void association::estimate_odometry(const sensor_msgs::LaserScan& scan_new, const sensor_msgs::LaserScan& scan_old)
{
  int i;
  float j;
  std::vector<Point*> staticPointCloud;
  std::vector<Point*> dynamicPointCloud;

  int max_itterations = round((scan_new.angle_max - scan_new.angle_min)/scan_new.angle_increment);

  for(i=0; i < max_itterations; i++)
  {
    //polygon_point_list.push_back(clustered_point);
    j = float(i)*scan_new.angle_increment+scan_new.angle_min;

    if(!isnan(scan_old.ranges[i]) && isnan(scan_new.ranges[i]))
    {
      staticPointCloud.push_back(new Point(scan_old.ranges[i]*cos(j), scan_old.ranges[i]*sin(j), 0.0f));
      dynamicPointCloud.push_back(new Point(scan_new.ranges[i]*cos(j), scan_new.ranges[i]*sin(j), 0.0f));

    }

  }
    //use iterative closest point to transform the dynamic point cloud to best align the static point cloud.
    icp(dynamicPointCloud, staticPointCloud);

 
    float alignmentError = 0.0f;
    for (int i = 0; i < dynamicPointCloud.size(); i++)
    {
      alignmentError += pow(dynamicPointCloud[i]->pos[0] - staticPointCloud[i]->pos[0], 2.0f);
      alignmentError += pow(dynamicPointCloud[i]->pos[1] - staticPointCloud[i]->pos[1], 2.0f);
      alignmentError += pow(dynamicPointCloud[i]->pos[2] - staticPointCloud[i]->pos[2], 2.0f);
    } 

    alignmentError /= (float)dynamicPointCloud.size();

    printf("Alignment Error: %0.5f \n", alignmentError);
    

    staticPointCloud.clear();
    dynamicPointCloud.clear();




}*/