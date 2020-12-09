#include "association.hpp"


association::association()
{


}

/* Associates objects with trackers*/
void association::association()
{
  int i,j,m = 0;
  double dt,time = scan_data_.header.stamp.sec + scan_data_.header.stamp.nsec*pow(10, -9);
  float s1,s2,s3,s4,s5,s6,similarity,sim_adj,x_dot,y_dot, sum[3];

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
          calculateVel(j, i,sum);
          s1 = sim_adj_angle*abs((trackers[i].average_angle) - (object_attributes_list[j].average_angle));
          s2 = sim_adj_dist*abs((trackers[i].longest_size) - (object_attributes_list[j].longest_size));
          s3 = sim_adj_side*abs((trackers[i].sides_amount) - (object_attributes_list[j].sides_amount));  
          s4 = sim_adj_xpos*abs((trackers[i].tracker.x_hat(0)) - (object_attributes_list[j].estimated_x));
          s5 = sim_adj_ypos*abs((trackers[i].tracker.x_hat(2)) - (object_attributes_list[j].estimated_y));

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
      /*
      x_dot = (trackers[m].tracker.x_hat(0)-object_attributes_list[j].estimated_x)/float(dt);
      y_dot = (trackers[m].tracker.x_hat(2)-object_attributes_list[j].estimated_y)/float(dt);
      */
      calculateVel(j, m,sum);
      x_dot = sum[1];
      y_dot = sum[2];
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




/* Estimates the movment between an object a a tracker*/
void association::calculateVel(int objectnr, int trackernr,float *sum)
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
    for(i=0;i < (int)(object_attributes_list[objectnr].points.points.size());i++)
    {
      x_o = object_attributes_list[objectnr].points.points[i].x;
      y_o = object_attributes_list[objectnr].points.points[i].y;
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