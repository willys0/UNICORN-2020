#ifndef __R2100_INTERFACE_H_
#define __R2100_INTERFACE_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

class R2100Interface {
    public:
        R2100Interface(ros::NodeHandle nh);

        void setLidarScanRanges(const std::vector<float>& ranges);

        void publish();
        void startTimer();

    protected:
        void initLidarMsg();

        void lidarPubTimeout(const ros::TimerEvent& e);


    private:
        ros::NodeHandle nh_;

        ros::Publisher lidar_pub_;

        ros::Timer lidar_pub_timer_;

        sensor_msgs::LaserScan lidar_scan_msg_;

        float lidar_pub_freq_;
        int   n_measures_;
};


#endif // __R2100_INTERFACE_H_
