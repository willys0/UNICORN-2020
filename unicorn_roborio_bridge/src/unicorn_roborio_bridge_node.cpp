#include <ros/ros.h>

#include <algorithm>

#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>

#include <unicorn_roborio_bridge/RioMasterMsg.h>

ros::Subscriber rio_sub;

ros::Publisher lift_state_pub;
ros::Publisher rear_lidar_pub;
// ros::Publisher  uwb_pos_pub;

std_msgs::Int32        lift_state;
float                  lift_freq;

sensor_msgs::LaserScan lidar_scan;
float                  lidar_freq;
// geometry_msgs::Point uwb_pos;

int n_measures;

void rio_cb(const unicorn_roborio_bridge::RioMasterMsgConstPtr& msg) {

    lift_state.data = msg->lift_state;

    // uwb_pos    = msg->uwb_pos;

    for(int i = 0; i < n_measures; i++) {
        // TODO: Maybe we can do the calculation on the RIO instead
        lidar_scan.ranges[i] = msg->lidar_ranges[i] / 1000.0;
    }

    lidar_scan.header.stamp = ros::Time::now();
    lidar_scan.header.seq++;
}

void lift_pub_timeout(const ros::TimerEvent& e) {
    lift_state_pub.publish(lift_state);
}

void lidar_pub_timeout(const ros::TimerEvent& e) {
    rear_lidar_pub.publish(lidar_scan);
}

std_msgs::Int32 init_lift_msg(void) {
    std_msgs::Int32 msg;

    ros::NodeHandle nh("lift");

    msg.data = 0;

    nh.param<float>("publish_frequency", lift_freq, 10.0);
    
    return msg;
}

sensor_msgs::LaserScan init_lidar_msg(void) {

    sensor_msgs::LaserScan msg;

    ros::NodeHandle nh("r2100");

    msg.header.frame_id = "rear_laser";

    // msg.angle_min =  -0.767945; //-44.0;
    nh.param<float>("angle_min", msg.angle_min, -0.698131701);
    nh.param<float>("angle_max", msg.angle_max, 0.767945);
    nh.param<float>("angle_increment", msg.angle_increment, 0.139626);
    nh.param<float>("range_min", msg.range_min, 0.1);
    nh.param<float>("range_max", msg.range_max, 8.0);


    nh.param<float>("publish_frequency", lidar_freq, 30.0);

    msg.time_increment = 0;
    msg.scan_time = 0;

    nh.param<int>("n_measures", n_measures, 11);
    msg.ranges.resize(n_measures);

    // for(int i = 0; i < 11; i++) {
    //     //msg.ranges.push_back(1.0);
    // }

    return msg;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "unicorn_roborio_bridge_node");

    ros::NodeHandle nh;

    rear_lidar_pub = nh.advertise<sensor_msgs::LaserScan>("/rearLidar/scan", 10);
    lift_state_pub = nh.advertise<std_msgs::Int32>("/lift/state", 1);

    // TODO: Implement when we get UWBs
    // uwb_pos_pub =    nh.advertise("/uwb/position");

    lift_state = init_lift_msg();
    lidar_scan = init_lidar_msg();

    rio_sub = nh.subscribe("/RIO_publisher", 10, &rio_cb);

    ros::Timer lidar_timer = nh.createTimer(ros::Duration(1.0 / lidar_freq), &lidar_pub_timeout);
    ros::Timer lift_timer = nh.createTimer(ros::Duration(1.0 / lift_freq), &lift_pub_timeout);

    ros::spin();
}
