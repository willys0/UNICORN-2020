#include "scan_odom.hpp"


/* Makes Variables reconfigurable */
void dynamicReconfigCallback(unicorn_scan_odometry::scan_odomConfig& config, uint32_t level, scan_odom *data) {
  data->outlier_ratio = config.outlier_ratio;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "scan_odom_node");

  ROS_INFO("Started Lidar scan odometry node");
  scan_odom scan_odom_interface;
  
  dynamic_reconfigure::Server<unicorn_scan_odometry::scan_odomConfig> reconfig_server;
  reconfig_server.setCallback(boost::bind(&dynamicReconfigCallback, _1, _2, &scan_odom_interface));
  ros::Rate r(30.0);

  ros::spin();
}

/* Initiate varibles and callsbacks  */
scan_odom::scan_odom()
: n_("~")
{

  n_.param("outlier_ratio",outlier_ratio, 25.0f);
  n_.param("base_laser_frame",base_laser_frame, std::string("base_laser")); 
  n_.param("base_frame",base_frame, std::string("chassis_link")); 
  n_.param("odomframeid",odomframeid, std::string("odom_chassis")); 


  scan_sub_ = n_.subscribe("/scan", 10, &scan_odom::scanCallback, this);

  est_odom_pub_ = n_.advertise<nav_msgs::Odometry>("odom",10,false);

  tf2_ros::TransformListener tf2_listener(tf_buffer);
  Lidar2base = tf_buffer.lookupTransform(base_frame, base_laser_frame, ros::Time(0),ros::Duration(100));
 
  seq = 0;

}

/* Scan callback, each frame is one scan*/
void scan_odom::scanCallback(const sensor_msgs::LaserScan& scan)
{

    scan_data_ = scan;

    esitmate_odometry();
    seq++;
}


using namespace std;
// Uses ICP to estimate odometry
void scan_odom::esitmate_odometry(){

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
      M[(i-m)*dim+1] = scan_data_old.ranges[i]*sin(float(i)*scan_data_old.angle_increment+scan_data_old.angle_min)*multiplier;
      if(isnan(M[(i-m)*dim+0]) || isinf(M[(i-m)*dim+0]) || isnan(M[(i-m)*dim+1]) || isinf(M[(i-m)*dim+1]) )
         m++;    
    }

    Matrix R = Matrix::eye(dim);
    Matrix t(dim,1);
    
    // run point-to-plane ICP (-1 = no outlier threshold)
    IcpPointToPlane icp(M,max_itterations_old-m,dim);
    double residual = icp.fit(T,max_itterations_new-j,R,t,(max_itterations_new-j)*outlier_ratio);
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
    tf2_ros::TransformListener tf2_listener(tf_buffer2);
    geometry_msgs::TransformStamped start_pose = tf_buffer2.lookupTransform(odomframeid, base_frame, ros::Time(0),ros::Duration(5));
    esitmated_odometry.header.frame_id = odomframeid;
    esitmated_odometry.pose.pose.position.x = start_pose.transform.translation.x;
    esitmated_odometry.pose.pose.position.y = start_pose.transform.translation.y;
    esitmated_odometry.pose.pose.position.z = start_pose.transform.translation.z;

    esitmated_odometry.pose.pose.orientation.x = start_pose.transform.rotation.x;
    esitmated_odometry.pose.pose.orientation.y = start_pose.transform.rotation.y;
    esitmated_odometry.pose.pose.orientation.z = start_pose.transform.rotation.z;
    esitmated_odometry.pose.pose.orientation.w = start_pose.transform.rotation.w;
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
