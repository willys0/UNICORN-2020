#include "association.hpp"


/* Sets Standard parameters */
association::association()
{
	sim_adj_dist = &sim_adj_dist_std;
	sim_adj_angle = &sim_adj_angle_std;
	sim_adj_side = &sim_adj_side_std;
	sim_adj_xpos = &sim_adj_xpos_std;
	sim_adj_ypos = &sim_adj_ypos_std;
	sim_adj_posdiff = &sim_adj_posdiff_std;
	max_similarty_deviation = &max_similarty_deviation_std;
	CONFIRMED_TRACK = &CONFIRMED_TRACK_std;
	TRACKER_LIFE = &TRACKER_LIFE_std;
}

/* Updateds adjustable parameters to custom values */
void association::association_setvar(int *CONFIRMED_TRACK_p, int *TRACKER_LIFE_p,float *max_similarty_deviation_p, float *sim_adj_dist_p,float *sim_adj_angle_p,float *sim_adj_side_p,float *sim_adj_xpos_p,float *sim_adj_ypos_p,float *sim_adj_posdiff_p){
  
  CONFIRMED_TRACK = CONFIRMED_TRACK_p;
  TRACKER_LIFE = TRACKER_LIFE_p;
  max_similarty_deviation = max_similarty_deviation_p; 
  sim_adj_dist = sim_adj_dist_p;
  sim_adj_angle = sim_adj_angle_p;
  sim_adj_side = sim_adj_side_p;
  sim_adj_xpos = sim_adj_xpos_p;
  sim_adj_ypos = sim_adj_ypos_p;
  sim_adj_posdiff = sim_adj_posdiff_p;
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

    trackers[i].age++;
    trackers[i].last_seen++;

    dt = time - trackers[i].time; 
    for(j=0; j<objects_size; j++)
    {
      if(object_attributes_list[j].polygon.points.size() >= 1){
        
        s1 = *sim_adj_angle*abs((trackers[i].average_angle) - (object_attributes_list[j].average_angle));
        s2 = *sim_adj_dist;
        if((trackers[i].length) > 0 && (object_attributes_list[j].length) > 0)
        {
          s2 = *sim_adj_dist*abs((trackers[i].length) - (object_attributes_list[j].length));
          if((trackers[i].width) > 0 && (object_attributes_list[j].width) > 0)
          {
            s2 /=2;
            s2 = *sim_adj_dist*abs((trackers[i].width) - (object_attributes_list[j].width))/(2);
          }
        }
          
        s3 = *sim_adj_side*abs((trackers[i].sides_amount) - (object_attributes_list[j].sides_amount));  
        s4 = *sim_adj_xpos*abs((trackers[i].tracker.x_hat(0)) - (object_attributes_list[j].position.x));
        s5 = *sim_adj_ypos*abs((trackers[i].tracker.x_hat(2)) - (object_attributes_list[j].position.y));

        calculateVel(object_attributes_list[j], i,sum);

        if(sum[0] < 0.00001)
            sum[0] = 1;

        s6 = *sim_adj_posdiff*abs(sum[0]);
        s6 = *sim_adj_posdiff;
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
          s6 = *sim_adj_posdiff;
        }
        similarity += s3 + s4 + s5 + s6;
        similarity /= (*sim_adj_dist + *sim_adj_angle + *sim_adj_side + *sim_adj_xpos + *sim_adj_ypos + *sim_adj_posdiff);
        // Incentices objects to be assigned to confirmed trackers
        if(trackers[i].age > *CONFIRMED_TRACK)
          similarity = similarity*0.7 + similarity*0.2*(*TRACKER_LIFE - (trackers[i].last_seen+1))/ *TRACKER_LIFE;
        if(!isnan(similarity))
          object_match_ratio[i][j] = double(similarity); 

        //ROS_INFO("Test sim %f - Tracker %d object %d , sim_prev %f velx %f vely %f, change yaw %f change x %f change y %f",similarity,i,j,sum[0],sum[1],sum[2],yaw_change, odom_change_x,odom_change_y);
        //ROS_INFO("Size cluster %d - size tracker cluster %d ",(int)object_attributes_list[j].polygon.points.size(), (int)trackers[i].cluster.points.size());
        //ROS_INFO("last seen %d age %d",trackers[i].last_seen, trackers[i].age);
      }
    }  
    
  }

  /* Hungarian algorithm to find the best match for the trackers */
  HungarianAlgorithm HungAlgo;
  vector<int> assignment;
  double cost = HungAlgo.Solve(object_match_ratio, assignment);


  /**/
  /* Associates trackers according to cost*//**/
  for (m = 0; m < trackers.size(); m++)
  {
    if((object_match_ratio[m][assignment[m]]) < *max_similarty_deviation)
    {
      geometry_msgs::Point new_trace;
      dt = time - trackers[m].time;
      object_match[assignment[m]] = 1; 
      trackers[m].last_seen = 0; 
      j = assignment[m];

      trackers[m].average_angle = object_attributes_list[j].average_angle;
      trackers[m].length = object_attributes_list[j].length;
      trackers[m].sides_amount = object_attributes_list[j].sides_amount;

      geometry_msgs::Point point;
      update_tracker(object_attributes_list[j], m,dt);

      new_trace.x = object_attributes_list[j].position.x;
      new_trace.y = object_attributes_list[j].position.y;
      new_trace.z = dt;
      trackers[m].trace.push_back(new_trace);
 
      trackers[m].points.points.clear();
     
      for(i = 0; i < object_attributes_list[j].polygon.points.size(); i++)
      {
        point.x = object_attributes_list[j].polygon.points[i].x;
        point.y = object_attributes_list[j].polygon.points[i].y;
        point.z = object_attributes_list[j].polygon.points[i].z;
        //point = transform_point(point, odometryData, BaseLaser2BaseFrame);
        geometry_msgs::Point32 point32;
        point32.x = point.x;
        point32.y = point.y;
        point32.z = point.z;
        trackers[m].points.points.push_back(point32);
      }
      //trackers[m].cluster.points.clear();
      //trackers[m].cluster = object_attributes_list[j].polygon;
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
    ROS_INFO("Tracker %d, last seen %d age %d - total life %d, confirmed %d",i,trackers[i].last_seen, trackers[i].age, *TRACKER_LIFE, *CONFIRMED_TRACK);

    if((trackers[i].last_seen > (*TRACKER_LIFE)) || (trackers[i].age < *CONFIRMED_TRACK && trackers[i].last_seen > 1))
    {

      trackers.erase(trackers.begin() + i);
      
      ROS_INFO("Tracker removed %d - trackers %d",i, (int)trackers.size());

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
  ROS_INFO("Number of trackers %d",(int)trackers.size());


  update_position();
}




/* Estimates the movment between an object a a tracker*/
void association::calculateVel(shape_extraction::object_attributes object, int trackernr,float *sum)
{
  int i,j;

  float x_t,y_t,x_o,y_o,distance;
  double roll,pitch,yaw;

  geometry_msgs::Polygon points;
  geometry_msgs::Point32 point1;
  geometry_msgs::Point point;

  point1.z = 10;
  point1.x = 0;
  point1.y = 0;
  sum[0] = 0;
  sum[1] = 0;
  sum[2] = 0;

  // Checks minimum movent between each point recorded in the tracker and the repective object
  for(j=0;j < (int)(trackers[trackernr].points.points.size()); j++)
  {
    x_t = trackers[trackernr].points.points[j].x*cos(yaw);
    y_t = trackers[trackernr].points.points[j].y*cos(yaw);
    point1.z = 10;
    point1.x = 0;
    point1.y = 0;

    //ROS_INFO("Point %d ",j,trackers[trackernr].cluster.points.x,trackers[trackernr].cluster.points.y);
    for(i=0;i < (int)(object.cluster.size());i++)
    {
      x_o = object.cluster[i].x;
      y_o = object.cluster[i].y;
      distance = sqrt(pow(x_o-x_t,2)+pow(y_o-y_t,2));
      // new positon
      if(distance < point1.z || i == 0){
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
  new_trackers.length = object.length;
  new_trackers.width = object.width;
  new_trackers.sides_amount = object.sides_amount;
  new_trackers.time = time;
  new_trackers.last_seen = 0;
  new_trackers.tracker.init();

  geometry_msgs::Point point;
  new_trackers.tracker.x_hat << object.position.x, 0,object.position.y, 0;
  new_trackers.tracker.y << object.position.x, 0, object.position.y, 0;

  geometry_msgs::Point new_trace;
  new_trace.x = object.position.x;
  new_trace.y = object.position.y;
  new_trace.z = 0;
  new_trackers.trace.push_back(new_trace);

  // Set Random Colors
  new_trackers.color[0] = 1;
  new_trackers.color[1] = float(rand() % 80 + 20)/100;
  new_trackers.color[2] = float(rand() % 80 + 20)/100;
  new_trackers.color[3] = float(rand() % 80 + 20)/100;
  new_trackers.points.points.clear();
  //new_trackers.cluster.points.clear();
  //new_trackers.cluster = object.polygon;
  int i;
  for(i = 0; i < object.polygon.points.size(); i++)
  {
    point.x = object.polygon.points[i].x;
    point.y = object.polygon.points[i].y;
    point.z = object.polygon.points[i].z;
    //point = transform_point(point, odometryData, BaseLaser2BaseFrame);
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

/* Updates a tracker with a new measured input */
void association::update_tracker(shape_extraction::object_attributes object,int trackerID,double dt)
{
  float x_dot = (trackers[trackerID].tracker.x_hat(0)-object.position.x)/float(dt);
  float y_dot = (trackers[trackerID].tracker.x_hat(2)-object.position.y)/float(dt);
  int m = 10;

  if(trackers[trackerID].trace.size() >= m)
  {
    x_dot = 0;
    y_dot = 0;
    int i = trackers[trackerID].trace.size()-1;
    std::cout <<  "mid_est translation x:" << (trackers[trackerID].trace[i-1].x - trackers[trackerID].trace[i].x) /float(trackers[trackerID].trace[i].z)  << " y" << (trackers[trackerID].trace[i-1].y - trackers[trackerID].trace[i].y) /float(trackers[trackerID].trace[i].z) << std::endl;
    for(i = trackers[trackerID].trace.size()-1;i > trackers[trackerID].trace.size()-m;i--)
    {
      x_dot += (trackers[trackerID].trace[i-1].x - trackers[trackerID].trace[i].x) /float(trackers[trackerID].trace[i].z);
      y_dot += (trackers[trackerID].trace[i-1].y - trackers[trackerID].trace[i].y) /float(trackers[trackerID].trace[i].z);
    }
    x_dot /= m;
    y_dot /= m;
  }

  //float sum[3];
  //calculateVel(object, trackerID,sum);
  //std::cout <<  "xmove_out translation x:" << x_dot << " y" << y_dot << std::endl;
  //std::cout <<  "move translation x:" << sum[1]/dt  << " y" << sum[2]/dt << std::endl;
  //std::cout <<  "med translation x:" << (sum[1]/dt+x_dot)/2  << " y" << (sum[2]/dt+y_dot)/2 << std::endl;

  trackers[trackerID].tracker.y << object.position.x, x_dot, object.position.y,y_dot;
  
}

/* Transforms a point  from local coordinates to global coordinates */
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
  position.x = x_t*cos(yaw) - y_t*sin(yaw);
  position.y = y_t*cos(yaw) + x_t*sin(yaw);
  
  position.x += odometryData.pose.pose.position.x;
  position.y += odometryData.pose.pose.position.y;
  position.z = 0;

  return position;
  // transform to map
  //tf2::doTransform(point,point,odom2map);
}

/* Estimates moment of clusteres with ICP - commented out due to stabiltiy problems*/
/*
// Uses ICP to estimate odometry
void association::estimate_moment(int trackerid, const shape_extraction::object_attributes& object, double dt){

  int i,m;
  int multiplier = 1000; // Solves issue with rounding error in ICP library

  int32_t dim = 2;  

  int max_itterations_new = object.polygon.points.size();
  int max_itterations_old = trackers[trackerid].cluster.points.size();
  double* M = (double*)calloc(dim*trackers[trackerid].cluster.points.size(),sizeof(double));
  double* T = (double*)calloc(dim*object.polygon.points.size(),sizeof(double));
  //double* T = (double*)calloc(dim*object.cluster.size(),sizeof(double));


   
  for(i = 0; i < max_itterations_old; i++)
  {
    // Transfor to Cartesian Coordinates
    M[(i)*dim+0] = trackers[trackerid].cluster.points[i].x;
    M[(i)*dim+1] = trackers[trackerid].cluster.points[i].y;
  }

  for(i = 0; i < max_itterations_new; i++)
  {
    // Transfor to Cartesian Coordinates
    T[(i)*dim+0] = object.polygon.points[i].x;
    T[(i)*dim+1] = object.polygon.points[i].y;

  }
    
  Matrix R = Matrix::eye(dim);
  Matrix t(dim,1);
  
  // run point-to-plane ICP (-1 = no outlier threshold)
  IcpPointToPlane icp(M,max_itterations_old,dim);
  double residual = icp.fit(T,max_itterations_new,R,t,-1);
  double rotation[4], rotation_val, translation[2];
  R.getData(rotation);
  t.getData(translation);
  rotation_val = atan2(rotation[1],rotation[0]);

  free(M);
  free(T);

  std::cout <<  "icp translation x:" << translation[0]/dt  << " y" << translation[1]/dt << std::endl;
    //std::cout <<  "rotation :" << esitmated_odometry.pose.pose.orientation.x << " "  << esitmated_odometry.pose.pose.orientation.y << " " << esitmated_odometry.pose.pose.orientation.z  << " " << esitmated_odometry.pose.pose.orientation.w<< std::endl;
  
}*/