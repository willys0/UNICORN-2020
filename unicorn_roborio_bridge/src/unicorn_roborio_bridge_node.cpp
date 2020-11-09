#include <ros/ros.h>

#include <algorithm>

#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>

#include <unicorn_roborio_bridge/RioMasterMsg.h>

ros::Subscriber rio_sub;

ros::Publisher lift_state_pub;
ros::Publisher rear_lidar_pub;
// ros::Publisher  uwb_pos_pub;

int                    lift_state;
sensor_msgs::LaserScan lidar_scan;
// geometry_msgs::Point uwb_pos;


void rio_cb(const unicorn_roborio_bridge::RioMasterMsgConstPtr& msg) {

    lift_state = msg->lift_state;

    // uwb_pos    = msg->uwb_pos;

    lidar_scan.ranges = msg->lidar_ranges;
    // std::reverse(msg->lidar_ranges.begin(), msg->lidar_ranges.end());
    // TODO: Update header
    lidar_scan.header.stamp = ros::Time::now();
    lidar_scan.header.seq++;
}

void lift_pub_timeout(const ros::TimerEvent& e) {
    static std_msgs::Int32 msg;

    lift_state_pub.publish(msg);
}

void lidar_pub_timeout(const ros::TimerEvent& e) {
    rear_lidar_pub.publish(lidar_scan);
}

sensor_msgs::LaserScan init_lidar_msg(void) {

    sensor_msgs::LaserScan msg;

    msg.header.frame_id = "rear_laser";

    // msg.angle_min =  -0.767945; //-44.0;
    msg.angle_min =  -0.698131701;
    msg.angle_max =   0.767945; // 44.0;
    msg.angle_increment = 0.139626; // 8.0;

    msg.range_min = 0.1;
    // Not sure about this
    msg.range_max = 8.0;

    msg.time_increment = 0;
    msg.scan_time = 0;

    for(int i = 0; i < 11; i++) {
        //msg.ranges.push_back((float)(i + 1) * 0.2);
        msg.ranges.push_back(1.0);
    }

    return msg;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "unicorn_roborio_bridge_node");

    ros::NodeHandle nh;

    rear_lidar_pub = nh.advertise<sensor_msgs::LaserScan>("/tmp/rearLidar/scan", 10);
    lift_state_pub = nh.advertise<std_msgs::Int32>("/lift/state", 1);

    // TODO: Implement when we get UWBs
    // uwb_pos_pub =    nh.advertise("/uwb/position");

    lidar_scan = init_lidar_msg();

    rio_sub = nh.subscribe<unicorn_roborio_bridge::RioMasterMsg>("/RIO_publisher", 10, &rio_cb);

    ros::Timer lidar_timer = nh.createTimer(ros::Duration(1.0 / 100.0), &lidar_pub_timeout);
    ros::Timer lift_timer = nh.createTimer(ros::Duration(1.0 / 10.0), &lift_pub_timeout);

    ros::spin();
}
