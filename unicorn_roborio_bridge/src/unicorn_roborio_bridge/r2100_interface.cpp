#include <unicorn_roborio_bridge/r2100_interface.h>

R2100Interface::R2100Interface(ros::NodeHandle nh) :
    nh_(nh)
{
    initLidarMsg();

    lidar_pub_ = nh.advertise<sensor_msgs::LaserScan>("/rearLidar/scan", 10);
}

void R2100Interface::setLidarScanRanges(const std::vector<float>& ranges) {
    for(int i = 0; i < n_measures_; i++) {
        // TODO: Maybe we can do the calculation on the RIO instead
        lidar_scan_msg_.ranges[i] = ranges[i] / 1000.0;
    }

    lidar_scan_msg_.header.stamp = ros::Time::now();
    lidar_scan_msg_.header.seq++;
   
}

void R2100Interface::publish() {
    lidar_pub_.publish(lidar_scan_msg_);
}


void R2100Interface::startTimer() {
    lidar_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / lidar_pub_freq_), &R2100Interface::lidarPubTimeout, this);
}

void R2100Interface::initLidarMsg() {

    lidar_scan_msg_.header.frame_id = "rear_laser";

    // msg.angle_min =  -0.767945; //-44.0;
    nh_.param<float>("angle_min", lidar_scan_msg_.angle_min, -0.698131701);
    nh_.param<float>("angle_max", lidar_scan_msg_.angle_max, 0.767945);
    nh_.param<float>("angle_increment", lidar_scan_msg_.angle_increment, 0.139626);
    nh_.param<float>("range_min", lidar_scan_msg_.range_min, 0.1);
    nh_.param<float>("range_max", lidar_scan_msg_.range_max, 8.0);


    nh_.param<float>("publish_frequency", lidar_pub_freq_, 30.0);

    lidar_scan_msg_.time_increment = 0;
    lidar_scan_msg_.scan_time = 0;

    nh_.param<int>("n_measures", n_measures_, 11);
    lidar_scan_msg_.ranges.resize(n_measures_);

    // for(int i = 0; i < 11; i++) {
    //     //lidar_scan_msg_.ranges.push_back(1.0);
    // }

    ROS_INFO("[R2100 settings] angle_min: %.2f, angle_max: %.2f, angle_increment: %.2f, range_min: %.2f, range_max: %.2f, publish_frequency: %.2f, n_measures: %d",
        lidar_scan_msg_.angle_min, lidar_scan_msg_.angle_max, lidar_scan_msg_.angle_increment, lidar_scan_msg_.range_min, lidar_scan_msg_.range_max, lidar_pub_freq_, n_measures_);

}

void R2100Interface::lidarPubTimeout(const ros::TimerEvent& e) {
    publish();
}
