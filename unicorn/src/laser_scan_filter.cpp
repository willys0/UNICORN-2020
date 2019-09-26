// Filter out unwanted angles from the LIDAR readings

#include <unicorn/laser_scan_filter.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");
  ROS_INFO("Started laser filter");
  LaserFilter laser_filter;
  ros::Rate r(15.0);
  //while(ros::ok())
  //{
    //laser_filter.publishScan();
    
   // r.sleep();
  //}
  ros::spin();
}

LaserFilter::LaserFilter()
{
  if (!n_.getParam("frame_id", chassis_frame_))
  {
    chassis_frame_ = "base_link";
  }
  ROS_INFO("[laser_filter]: chassis_frame: %s", chassis_frame_.c_str());
  std::string scan_topic;
  if (!n_.getParam("scan_topic", scan_topic))
  {
    scan_topic = "/scan_filtered";
  }
  scan_sub_ = n_.subscribe("/scan", 1, &LaserFilter::scanCallback, this);
  scan_pub_ = n_.advertise<sensor_msgs::LaserScan>(scan_topic.c_str(), 1);
  if (n_.getParam("/laser_filter/lower_angle", lower_angle_))
  {
    ROS_INFO("[laser_filter]: Lower angle threshold: %f", lower_angle_);
  }
  else
  {
    ROS_WARN("[laser_filter]: Lower angle threshold not set");
    lower_angle_ = -1.57;
  }
  if (n_.getParam("/laser_filter/upper_angle", upper_angle_))
  {
    ROS_INFO("[laser_filter]: Upper angle threshold: %f", upper_angle_);
  }
  else
  {
    ROS_WARN("[laser_filter]: Upper angle threshold not set");
    upper_angle_ = 1.57;
  }
  float heading;
  if(getLaserPose(heading))
  {
    ROS_INFO("[laser_filter] heading: %f", heading);
  }
  else
  {
    ROS_ERROR("[laser_filter] Transform to base_laser not available");
    heading = 0;
  }
//  lower_angle_ -= heading;
//  upper_angle_ -= heading;
  
}

int LaserFilter::getLaserPose(float& heading)
{
  tf::StampedTransform transform;
  ros::Time now = ros::Time::now();
  try
  {
    tf_listener_.waitForTransform(chassis_frame_.c_str(),
                  "base_laser", 
                  now,
                  ros::Duration(30.0));
    tf_listener_.lookupTransform(chassis_frame_.c_str(),
                  "base_laser", 
                  now,
                  transform);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
    return 0;
  }
  tf::Quaternion quat = transform.getRotation();
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  heading = yaw;
  return 1;
}

LaserFilter::~LaserFilter()
{

}

void LaserFilter::publishScan()
{
  scan_pub_.publish(scan_);
}

void LaserFilter::scanCallback22(const sensor_msgs::LaserScan& input_scan)
{
    float close = 1000;
    int index = -1;
		for (unsigned int i = 0; i < input_scan.ranges.size(); ++i)
		  {
			if (input_scan.ranges[i] < close)
			{
				close =input_scan.ranges[i];
				index = i; 
			}
		}
		ROS_INFO("The closest one is on %i value: %2.4f",index,close);
  
}

void LaserFilter::scanCallback(const sensor_msgs::LaserScan& input_scan)
{
  

  scan_.ranges.resize(input_scan.ranges.size());
  scan_.intensities.resize(input_scan.intensities.size());

  double start_angle = input_scan.angle_min;
  double current_angle = input_scan.angle_min;
  ros::Time start_time = input_scan.header.stamp;
  unsigned int count = 0;
  //ROS_INFO("Start = %f", start_angle);
  //loop through the scan and truncate the beginning and the end of the scan as necessary
  for (unsigned int i = 0; i < input_scan.ranges.size(); ++i)
  {
    //Since the lidar has a f* upped coordinate system some magic needs to happen.
/*	 
    if(upper_angle_ < lower_angle_){
	/* This case presents itself when trying to read data in front of the robot. *
	 * What happens here is that data points behind the robot are filtered away. *
		if(current_angle <= upper_angle_ or current_angle >= lower_angle_){
			scan_.ranges[count] = input_scan.ranges[i];
			//make sure  that we don't update intensity data if its not available
			if (input_scan.intensities.size() > i)
				scan_.intensities[count] = input_scan.intensities[i];
			count++;
			current_angle += input_scan.angle_increment;
		}
		else{
			current_angle += input_scan.angle_increment;
		}
    }*/
//    else{
		//wait until we get to our desired starting angle
            if(start_angle < lower_angle_) 
	    {
	      start_angle += input_scan.angle_increment;
	      current_angle += input_scan.angle_increment;
	      start_time += ros::Duration(input_scan.time_increment);
	    }
	    else 
	    {
	      scan_.ranges[count] = input_scan.ranges[i];

	      //make sure  that we don't update intensity data if its not available
	      if (input_scan.intensities.size() > i)
			scan_.intensities[count] = input_scan.intensities[i];
		  count++;
		//check if we need to break out of the loop, basically if the next increment will put us over the threshold
		if (current_angle + input_scan.angle_increment > upper_angle_) 
		{
		  break;
		}

//	          ROS_INFO("Current angle = %f, upper_angle = %f, lower_angle = %f", current_angle, upper_angle_, lower_angle_);
	      current_angle += input_scan.angle_increment;
//	}
  }
	
}
 
  

  //populate the LaserScan message
  scan_.header.frame_id = input_scan.header.frame_id;
  scan_.time_increment = input_scan.time_increment;
  scan_.angle_increment = input_scan.angle_increment;
  scan_.scan_time = input_scan.scan_time;
  scan_.range_min = input_scan.range_min;
  scan_.range_max = input_scan.range_max;
  scan_.header.stamp = start_time;
  scan_.angle_min = start_angle;
  scan_.angle_max = current_angle;
 
  scan_.ranges.resize(count);

  if (input_scan.intensities.size() >= count)
  {
    scan_.intensities.resize(count);
  }
  //laser_filter.publishScan();
}
