// Reads Ultra sonic sensor data from the USB port.

#include <unicorn/range_sensor_driver.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "range_sensor_driver");
	RangeDriver range_driver;
	ros::Rate r(100);
	while(ros::ok())
	{
	  range_driver.readLine();
      range_driver.publishData();
	  ros::spinOnce();
	  r.sleep();
	}
	return 0;
}

RangeSensor::RangeSensor(std::string sensor_topic, std::string sensor_frame)
: TOPIC(sensor_topic)
{
	range_pub_ = n_.advertise<sensor_msgs::Range>(TOPIC.c_str(), 0, this);
	range_msg_.header.frame_id = sensor_frame.c_str();
	range_msg_.radiation_type = sensor_msgs::Range::ULTRASOUND;
	range_msg_.min_range = 0.2;
	range_msg_.max_range = 8.0;
	range_msg_.field_of_view = 15*M_PI/180;
}

void RangeSensor::setRange(float range)
{
	range_msg_.header.stamp = ros::Time::now();
	range_msg_.range = range;
}

void RangeSensor::publishRange()
{	
	range_pub_.publish(range_msg_);
}

RangeDriver::RangeDriver()
{
	std::string serial_port;
	if(n_.getParam("range_sensor_driver/serial_port", serial_port))
	{
		SERIAL_PORT_ = serial_port;
		ROS_INFO("[range_sensor_driver] Trying serial port: %s", serial_port.c_str());
	}
	else
	{
		SERIAL_PORT_ = "/dev/ttyACM0";
	}
	file_.open(SERIAL_PORT_.c_str());
	if ( (file_.rdstate() & std::ifstream::failbit ) != 0 )
	{
		ROS_ERROR("[range_sensor_driver] Failed to find serial port!");
  		ros::shutdown();
	}
	std::vector<std::string> range_name_list;
	/*"/move_base/local_costmap/sonar_layer/topics"*/
    n_.getParam("range_sensor_driver/topics", range_name_list);
    if (range_name_list.size() > 0)
    {
    	for (int i = 0; i < range_name_list.size(); ++i)
    	{
    		ROS_INFO("[range_sensor_driver] frame_id: %s", std::string(std::string("base_")+range_name_list[i].substr(1,range_name_list[i].size())).c_str());
    		range_sensor_list_.push_back(new RangeSensor(range_name_list[i],
    			std::string("base_")+range_name_list[i].substr(1,range_name_list[i].size())));
    	}
    }
    else
    {
    	ROS_WARN("[range_driver] No list of range sensors specified (range_sensor_list: [s1,s2,s3,...,sn])");
    	if (file_ >> range_data_)
    	{
    		int i = 0, index = 0;
	    	while(index != range_data_.npos)
	    	{
	    		ROS_INFO("fisk");
	    		index = range_data_.find_first_of(':', index);
				index++;
				if (index != 0)
				{
		    		range_sensor_list_.push_back(new RangeSensor(std::string("/ultrasonic_")+boost::lexical_cast<std::string>(i),
		    			std::string("base_ultrasonic_")+boost::lexical_cast<std::string>(i)));
		    		i++;
	    		}
	    		else
	    		{
	    			range_sensor_list_.pop_back();
	    			return;
	    		}
	    	}
    	}
    	else
    	{
    		ROS_ERROR("[range_sensor_driver] Failed to read from serial port!");
    		ros::shutdown();
    	}
    	
    }
}
RangeDriver::~RangeDriver()
{
	file_.close();
}

void RangeDriver::readLine()
{
	if (file_ >> range_data_)
	{
		int i = 0, end_index = 0, start_index = 0;
		start_index = range_data_.find_first_of(':', start_index);
		start_index++;
		while(end_index != range_data_.npos)
		{
			end_index = start_index;
			end_index = range_data_.find_first_of(':', end_index);
			if (end_index >= start_index)
			{
				try
				{
					float test = boost::lexical_cast<float>(range_data_.substr(start_index,end_index-start_index));
				}
				catch(boost::bad_lexical_cast &)
				{
					ROS_WARN("[range_sensor_driver] Bad serial reading!");
					return;
				}
				range_sensor_list_[i]->setRange(boost::lexical_cast<float>(range_data_.substr(start_index,end_index-start_index)) / 100);
				i++;
				start_index = range_data_.find_first_of(':', start_index);
				start_index++;
			}
			else
			{
				return;
			}
			
		}
	}
}

void RangeDriver::publishData()
{
	for (int i = 0; i < range_sensor_list_.size(); ++i)
	{
		range_sensor_list_[i]->publishRange();
	}
}