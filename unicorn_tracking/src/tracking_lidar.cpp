#include "tracking_lidar.hpp"


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
  data->sim_adj_posdiff = config.sim_adj_posdiff;
  data->TRACKER_LIFE = config.TRACKER_LIFE;
  data->CONFIRMED_TRACK = config.CONFIRMED_TRACK;
  data->static_remove_ratio = config.static_remove_ratio;
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
  n_.param("max_dist_laser",max_dist_laser, 10.0f);

  n_.param("static_filter",static_filter, true);
  n_.param("Static_map_removal_tolerance",static_remove_dist, 4);
  n_.param("Static_map_removal_tolerance_ratio",static_remove_ratio, 0.5f);

  

  n_.param("polygon_tolerance",polygon_tolerance, 1.04f);
  n_.param("polygon_side_min_points_required",polygon_min_points, 4);
  n_.param("Cluster_Lower_Limit",min_size_cluster, 5);
  

  n_.param("min_twist_detection",min_twist_detection, 0.08f);

  n_.param("similarty_side_length_weight",sim_adj_dist, 1.0f);
  n_.param("similarty_side_angle_weigh",sim_adj_angle, 1.0f);
  n_.param("similarty_side_amount_weight",sim_adj_side, 3.0f);
  n_.param("similarty_track_xposition_weight",sim_adj_xpos, 5.0f);
  n_.param("similarty_track_yposition_weight",sim_adj_ypos, 5.0f);
  n_.param("similarty_previous_position_weight",sim_adj_posdiff, 5.0f);
  n_.param("TRACKER_LIFE",TRACKER_LIFE, 1000);
  n_.param("CONFIRMED_TRACK",CONFIRMED_TRACK, 100);
  

  n_.param("max_similarty_deviation",max_similarty_deviation, 1.5f);
  n_.param("map_topic",mapframeid, std::string("map")); //std::string("rear_laser")
  n_.param("base_laser_frame",base_laser_frame, std::string("base_laser")); 
  n_.param("base_frame",base_frame, std::string("chassis_link")); 
  n_.param("odomframeid",odomframeid, std::string("odom_chassis")); 

  
  odometry_sub_ = n_.subscribe("/odometry/filtered", 10, &tracking_lidar::odomCallback, this);
  scan_sub_ = n_.subscribe("/frontLidar/scan", 10, &tracking_lidar::scanCallback, this);

  // publishers
  object_pub_ = n_.advertise<costmap_converter::ObstacleArrayMsg>("/move_base/TebLocalPlannerROS/obstacles",10,false);
  marker_pub_ = n_.advertise<visualization_msgs::MarkerArray>("markerArray",10,false);
  marker_Arrow_pub_ = n_.advertise<visualization_msgs::MarkerArray>("markerArrowArray",10,false);
  est_odom_pub_ = n_.advertise<nav_msgs::Odometry>("icp/odom",10,false);

  tf2_ros::TransformListener tf2_listener(tf_buffer);
  Lidar2base = tf_buffer.lookupTransform(base_frame, base_laser_frame, ros::Time(0),ros::Duration(100));

  if(static_filter)
  {
    map_sub_ = n_.subscribe("/map", 10, &tracking_lidar::mapCallback, this);
  }else
    map_received = true;


  seq = 0;
  shape_interface.shape_extraction_setvar(&lambda, &max_dist_laser, &static_remove_dist, &static_remove_ratio, &min_size_cluster, &polygon_tolerance, &polygon_min_points);
  association_interface.association_setvar(&CONFIRMED_TRACK , &TRACKER_LIFE , &max_similarty_deviation , &sim_adj_dist ,&sim_adj_angle ,&sim_adj_side ,&sim_adj_xpos ,&sim_adj_ypos ,&sim_adj_posdiff);
  //initiate_Trackers();
}


/* Publishes data to topics */
void tracking_lidar::object_publisher()
{
  costmap_converter::ObstacleMsg object;
  costmap_converter::ObstacleArrayMsg object_array;
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markerArray, markerDArray;
  geometry_msgs::Point point, velocity, position;
  geometry_msgs::Point32 point32;
  tf2::Quaternion Quad;

  object.header.frame_id = mapframeid;
  
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
  for(i=0; i < association_interface.trackers.size(); i++)
  {
    if(association_interface.trackers[i].age > CONFIRMED_TRACK){
      float min_length_arrow = sqrt(pow(association_interface.trackers[i].tracker.x_hat(1),2) + pow(association_interface.trackers[i].tracker.x_hat(3),2));
      marker.type = 5;

      object.polygon.points.clear();
      for(m=0; m < association_interface.trackers[i].points.points.size();m++)
      {
        point.x = association_interface.trackers[i].points.points[m].x - position.x;
        point.y = association_interface.trackers[i].points.points[m].y - position.y;
        point.z = association_interface.trackers[i].points.points[m].z;
        velocity = transform_point_odometry(point, esitmated_odometry, odometry_data_);
        point32.x = point.x;
        point32.y = point.y;
        point32.z = point.z;
        object.polygon.points.push_back(point32);
      }
            

      if(min_length_arrow > min_twist_detection)
      {
        velocity.x  = association_interface.trackers[i].tracker.x_hat(1);
        velocity.y  = association_interface.trackers[i].tracker.x_hat(3); 
        velocity = transform_vel_odometry(velocity, esitmated_odometry, odometry_data_);
        object.velocities.twist.linear.x = velocity.x;
        object.velocities.twist.linear.y = velocity.y;
      }else{
        object.velocities.twist.linear.x = 0;
        object.velocities.twist.linear.y = 0;
      }

      Quad.setRPY(0,0,atan2(object.velocities.twist.linear.y,object.velocities.twist.linear.x));

      object_array.obstacles.push_back(object);

      marker.color.a = association_interface.trackers[i].color[0]*(TRACKER_LIFE - (association_interface.trackers[i].last_seen+1))/TRACKER_LIFE;
      marker.color.b = association_interface.trackers[i].color[1];
      marker.color.g = association_interface.trackers[i].color[2];
      marker.color.r = association_interface.trackers[i].color[3];
      position.x = association_interface.trackers[i].tracker.x_hat(0);
      position.y = association_interface.trackers[i].tracker.x_hat(2);
      position = transform_point_odometry(position, esitmated_odometry, odometry_data_);
      marker.pose.position.x = position.x;
      marker.pose.position.y = position.y;   
      marker.id = i;


      marker.points.clear();
      for(m =0; m < association_interface.trackers[i].points.points.size()-1; m++)
      {
        point.x = association_interface.trackers[i].points.points[m].x;
        point.y = association_interface.trackers[i].points.points[m].y;
        point.z = association_interface.trackers[i].points.points[m].z;
        point = transform_point_odometry(point, esitmated_odometry, odometry_data_);
        point.x -= position.x;
        point.y -= position.y;
        point.z = Lidar_Height;
        marker.points.push_back(point);
        point.x = association_interface.trackers[i].points.points[m+1].x;
        point.y = association_interface.trackers[i].points.points[m+1].y;
        point.z = association_interface.trackers[i].points.points[m+1].z;
        point = transform_point_odometry(point, esitmated_odometry, odometry_data_);
        point.x -= position.x;
        point.y -= position.y;
        point.z = Lidar_Height;
        marker.points.push_back(point);
      }
      markerArray.markers.push_back(marker);
     
      if((min_length_arrow > min_twist_detection) && association_interface.trackers[i].last_seen == 0)
      {
        marker.type = 0;
        marker.points.clear();
        point.x = 0;
        point.y = 0;
        point.z = 0;
        marker.points.push_back(point);
        point.x = velocity.x/min_length_arrow;
        point.y = velocity.y/min_length_arrow;
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
}

/* Scan callback, each frame is one scan*/
void tracking_lidar::scanCallback(const sensor_msgs::LaserScan& scan)
{

    scan_data_ = scan;
    scan_received = true;
    esitmate_odometry();
    if(map_received && odom_received && scan_received){
      

      shape_interface.adaptive_break_point(scan);
      std::cout << "Test clsuters:" << std::endl;
      std::cout << shape_interface.cluster_list.size() << std::endl;
      if(static_filter)
        shape_interface.static_map_filter(map_data_, odometry_data_, Lidar2base);
      
      shape_interface.transformObjects(esitmated_odometry,Lidar2base);
      shape_interface.polygon_extraction();
      std::cout << "Test objects:" << std::endl;
      std::cout << shape_interface.object_attributes_list.size() << std::endl;
      shape_interface.polygon_attribute_extraction();

      association_interface.associate(shape_interface.object_attributes_list, esitmated_odometry, Lidar2base, scan.header.stamp);

      object_publisher();
      scan_received = false; 
      
    }
    seq++;
}



// Uses ICP to estimate odometry
using namespace std;
void tracking_lidar::esitmate_odometry(){

  int i,m;
  int multiplier = 1000; // Solves issue with rounding error in ICP library
  esitmated_odometry.header.stamp = scan_data_.header.stamp;
  esitmated_odometry.header.seq = seq;

  if(seq > 0 && !isnan(esitmated_odometry.pose.pose.orientation.x))
  {
    int max_itterations_new = round((scan_data_.angle_max - scan_data_.angle_min)/scan_data_.angle_increment);
    int max_itterations_old = round((scan_data_old.angle_max - scan_data_old.angle_min)/scan_data_old.angle_increment);

    int32_t dim = 2;    
    double* M = (double*)calloc(dim*max_itterations_new,sizeof(double));
    double* T = (double*)calloc(dim*max_itterations_old,sizeof(double));

    int j = 0;
    for(i = 0; i < max_itterations_new; i++)
    {
      // Transfor to Cartesian Coordinates
      T[(i-j)*dim+0] = scan_data_.ranges[i]*cos(float(i)*scan_data_.angle_increment+scan_data_.angle_min)*multiplier;
      T[(i-j)*dim+1] = scan_data_.ranges[i]*sin(float(i)*scan_data_.angle_increment+scan_data_.angle_min)*multiplier;
      if(isnan(T[(i-j)*dim+0]) || isinf(T[(i-j)*dim+0]) || isinf(T[(i-j)*dim+1]) || isnan(T[(i-j)*dim+1]))
        j++;
    }
    int m = 0;
    for(i = 0; i < max_itterations_old; i++)
    {
      // Transfor to Cartesian Coordinates
      M[(i-m)*dim+0] = scan_data_old.ranges[i]*cos(float(i)*scan_data_old.angle_increment+scan_data_old.angle_min)*multiplier;
      M[(i-m)*dim+1]  = scan_data_old.ranges[i]*sin(float(i)*scan_data_old.angle_increment+scan_data_old.angle_min)*multiplier;
      if(isnan(M[(i-m)*dim+0]) || isinf(M[(i-m)*dim+0]) || isnan(M[(i-m)*dim+1]) || isinf(M[(i-m)*dim+1]) )
         m++;    
    }

    Matrix R = Matrix::eye(dim);
    Matrix t(dim,1);
    
    // run point-to-plane ICP (-1 = no outlier threshold)
    IcpPointToPlane icp(M,max_itterations_old-m,dim);
    double residual = icp.fit(T,max_itterations_new-j,R,t,50);
    double rotation[4], rotation_val, translation[2];
    R.getData(rotation);
    t.getData(translation);
    rotation_val = atan2(rotation[1],rotation[0]);

    free(M);
    free(T);

    double roll,pitch,yaw_est;
    tf::Quaternion q(esitmated_odometry.pose.pose.orientation.x,esitmated_odometry.pose.pose.orientation.y,esitmated_odometry.pose.pose.orientation.z,esitmated_odometry.pose.pose.orientation.w);
    tf::Matrix3x3 n(q);
    n.getRPY(roll,pitch,yaw_est);
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY( roll, pitch,yaw_est-rotation_val); 

    esitmated_odometry.pose.pose.orientation.x = myQuaternion.getX();
    esitmated_odometry.pose.pose.orientation.y = myQuaternion.getY();
    esitmated_odometry.pose.pose.orientation.z = myQuaternion.getZ();
    esitmated_odometry.pose.pose.orientation.w = myQuaternion.getW();

    esitmated_odometry.pose.pose.position.x += translation[0]/multiplier*cos(yaw_est-rotation_val) - translation[1]/multiplier*sin(yaw_est-rotation_val);
    esitmated_odometry.pose.pose.position.y += translation[0]/multiplier*sin(yaw_est-rotation_val) + translation[1]/multiplier*cos(yaw_est-rotation_val);

    est_odom_pub_.publish(esitmated_odometry);
    //std::cout <<  "translation x:" << esitmated_odometry.pose.pose.position.x  << " y" << esitmated_odometry.pose.pose.position.y << std::endl;
    //std::cout <<  "rotation :" << esitmated_odometry.pose.pose.orientation.x << " "  << esitmated_odometry.pose.pose.orientation.y << " " << esitmated_odometry.pose.pose.orientation.z  << " " << esitmated_odometry.pose.pose.orientation.w<< std::endl;
  }else{
    esitmated_odometry.header.frame_id = odomframeid;
    esitmated_odometry.pose.pose.position.x = odometry_data_.pose.pose.position.x;
    esitmated_odometry.pose.pose.position.y = odometry_data_.pose.pose.position.y;
    esitmated_odometry.pose.pose.position.z = odometry_data_.pose.pose.position.z;

    esitmated_odometry.pose.pose.orientation.x = odometry_data_.pose.pose.orientation.x;
    esitmated_odometry.pose.pose.orientation.y = odometry_data_.pose.pose.orientation.y;
    esitmated_odometry.pose.pose.orientation.z = odometry_data_.pose.pose.orientation.z;
    esitmated_odometry.pose.pose.orientation.w = odometry_data_.pose.pose.orientation.w;
    esitmated_odometry.pose.covariance = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                          0.0, 0.0, 0.0, 0.0, 0.0, 1.0
                                          };
  }
  scan_data_old = scan_data_;
}


geometry_msgs::Point tracking_lidar::transform_point_odometry(geometry_msgs::Point position, const nav_msgs::Odometry& odometryData_old,const nav_msgs::Odometry& odometryData_new)
{
  double roll,pitch,yaw_old,yaw_new,x_t,y_t;
  tf::Quaternion q(odometryData_old.pose.pose.orientation.x,odometryData_old.pose.pose.orientation.y,odometryData_old.pose.pose.orientation.z,odometryData_old.pose.pose.orientation.w);
  tf::Quaternion r(odometryData_new.pose.pose.orientation.x,odometryData_new.pose.pose.orientation.y,odometryData_new.pose.pose.orientation.z,odometryData_new.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  tf::Matrix3x3 n(r);
  m.getRPY(roll,pitch,yaw_old);
  n.getRPY(roll,pitch,yaw_new);

  position.x -= odometryData_old.pose.pose.position.x;
  position.y -= odometryData_old.pose.pose.position.y;

  x_t = position.x;
  y_t = position.y;
  position.x = x_t*cos(yaw_new-yaw_old) - y_t*sin(yaw_new-yaw_old);
  position.y = y_t*cos(yaw_new-yaw_old) + x_t*sin(yaw_new-yaw_old);

  position.x += odometryData_new.pose.pose.position.x;
  position.y += odometryData_new.pose.pose.position.y;
  position.z = 0;

  return position;
}

geometry_msgs::Point tracking_lidar::transform_vel_odometry(geometry_msgs::Point position, const nav_msgs::Odometry& odometryData_old,const nav_msgs::Odometry& odometryData_new)
{
  double roll,pitch,yaw_old,yaw_new,x_t,y_t;
  tf::Quaternion q(odometryData_old.pose.pose.orientation.x,odometryData_old.pose.pose.orientation.y,odometryData_old.pose.pose.orientation.z,odometryData_old.pose.pose.orientation.w);
  tf::Quaternion r(odometryData_new.pose.pose.orientation.x,odometryData_new.pose.pose.orientation.y,odometryData_new.pose.pose.orientation.z,odometryData_new.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  tf::Matrix3x3 n(r);
  m.getRPY(roll,pitch,yaw_old);
  n.getRPY(roll,pitch,yaw_new);

  x_t = position.x;
  y_t = position.y;
  position.x = x_t*cos(yaw_new-yaw_old) - y_t*sin(yaw_new-yaw_old);
  position.y = y_t*cos(yaw_new-yaw_old) + x_t*sin(yaw_new-yaw_old);
  

  return position;
}