#include <ros/ros.h>

#include <algorithm>

#include <actionlib/server/simple_action_server.h>

#include <std_msgs/Int32.h>
#include <sensor_msgs/LaserScan.h>

#include <unicorn_roborio_bridge/RioMasterMsg.h>
#include <unicorn_roborio_bridge/RunLiftAction.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#define RIO_LIFT_ACTION_STOP    0
#define RIO_LIFT_ACTION_PICKUP  1
#define RIO_LIFT_ACTION_DROPOFF 2

#define RIO_LIFT_STATE_IDLE  0
#define RIO_LIFT_STATE_ERROR 6

typedef actionlib::SimpleActionServer<unicorn_roborio_bridge::RunLiftAction> LiftActionServer;

ros::Subscriber rio_sub;

ros::Publisher lift_state_pub;
ros::Publisher rear_lidar_pub;
ros::Publisher uwb_pos_pub;

std_msgs::Int32        lift_state;
float                  lift_freq;

sensor_msgs::LaserScan lidar_scan;
float                  lidar_freq;

geometry_msgs::PoseWithCovarianceStamped uwb_pos;
float uwb_freq;

int n_measures;

void execute_lift(const unicorn_roborio_bridge::RunLiftGoalConstPtr& goal, LiftActionServer* as, ros::NodeHandle nh) {
    ros::Publisher run_lift_pub;
    std_msgs::Int32 msg;
    int dir;

    if(goal->direction == goal->DIRECTION_PICKUP) {
        dir = RIO_LIFT_ACTION_PICKUP;
        ROS_INFO("[Roborio Bridge] Running lift pickup routine.");
    }
    else if(goal->direction == goal->DIRECTION_DROPOFF) {
        dir = RIO_LIFT_ACTION_DROPOFF;
        ROS_INFO("[Roborio Bridge] Running lift dropoff routine.");
    }
    else {
        // TODO: Handle wrong input value
        ROS_ERROR("[Roborio Bridge] Invalid lift direction given to lifting action.");
        as->setAborted();
        return;
    }

    // Wait for lift to be idle
    ROS_INFO("[Roborio Bridge] Waiting for lift to be idle...");
    while(lift_state.data != RIO_LIFT_STATE_IDLE) {
        if(!ros::ok()) {
            as->setAborted();
            return;
        }
    }

    ROS_INFO("[Roborio Bridge] Sending command to lift.");
    run_lift_pub = nh.advertise<std_msgs::Int32>("/TX2_unicorn_picking_routine", 1);
    msg.data = dir;
    run_lift_pub.publish(msg);

    // Wait for lift to move out of idle state
    while(ros::ok() && lift_state.data == RIO_LIFT_STATE_IDLE) {
        if(as->isPreemptRequested()) {
            break;
        }

        // TODO: May need to retry sending message here

        ros::spinOnce();
    }

    ROS_INFO("[Roborio Bridge] Waiting for lift to finish...");

    // Wait for lift to be done
    while(ros::ok()) {
        if(as->isPreemptRequested()) {

            msg.data = RIO_LIFT_ACTION_STOP;
            run_lift_pub.publish(msg);

            ROS_INFO("[Roborio Bridge] Lift action preempted.");
            as->setPreempted();

            // TODO: Shall we reverse the lift back to initial position?
            break;
        }

        if(lift_state.data == RIO_LIFT_STATE_IDLE) {
            ROS_INFO("[Roborio Bridge] Lift action succeeded.");
            as->setSucceeded();
            break;
        }
        else if(lift_state.data == RIO_LIFT_STATE_ERROR) {
            // TODO: Handle error better
            ROS_ERROR("[Roborio Bridge] Lift action failed.");
            as->setAborted();
            break;
        }

        ros::spinOnce();
    }

    run_lift_pub.shutdown();

}

void rio_cb(const unicorn_roborio_bridge::RioMasterMsgConstPtr& msg) {

    lift_state.data = msg->lift_state;

    for(int i = 0; i < n_measures; i++) {
        // TODO: Maybe we can do the calculation on the RIO instead
        lidar_scan.ranges[i] = msg->lidar_ranges[i] / 1000.0;
    }

    lidar_scan.header.stamp = ros::Time::now();
    lidar_scan.header.seq++;

    uwb_pos.header.stamp = ros::Time::now();
    uwb_pos.header.seq++;

    uwb_pos.pose.pose.position = msg->uwb_pos;
}

void lift_pub_timeout(const ros::TimerEvent& e) {
    lift_state_pub.publish(lift_state);
}

void lidar_pub_timeout(const ros::TimerEvent& e) {
    rear_lidar_pub.publish(lidar_scan);
}

void uwb_pub_timeout(const ros::TimerEvent& e) {
    uwb_pos_pub.publish(uwb_pos);
}

std_msgs::Int32 init_lift_msg(void) {
    std_msgs::Int32 msg;

    ros::NodeHandle nh("~/lift");

    msg.data = 0;

    nh.param<float>("publish_frequency", lift_freq, 10.0);
    
    ROS_INFO("[Lift settings] publish_frequency: %.2f", 
        lift_freq);

    return msg;
}

sensor_msgs::LaserScan init_lidar_msg(void) {

    sensor_msgs::LaserScan msg;

    ros::NodeHandle nh("~/r2100");

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

    ROS_INFO("[R2100 settings] angle_min: %.2f, angle_max: %.2f, angle_increment: %.2f, range_min: %.2f, range_max: %.2f, publish_frequency: %.2f, n_measures: %d", 
        msg.angle_min, msg.angle_max, msg.angle_increment, msg.range_min, msg.range_max, lidar_freq, n_measures);
    return msg;
}

geometry_msgs::PoseWithCovarianceStamped init_uwb_msg() {
    geometry_msgs::PoseWithCovarianceStamped msg;

    ros::NodeHandle nh("~/uwb");

    // TODO: Do we need to assign a frame to the header?
    nh.param<std::string>("reference_frame", msg.header.frame_id, "");
    nh.param<float>("publish_frequency", uwb_freq, 30.0);

    // TODO: Incorporate orientation if we can get it later
    msg.pose.pose.orientation.x = 0;
    msg.pose.pose.orientation.y = 0;
    msg.pose.pose.orientation.z = 0;
    msg.pose.pose.orientation.w = 1;

    boost::array<double, 36UL> covariance({
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0
    });
    msg.pose.covariance.swap(covariance);

    ROS_INFO("[UWB settings] reference_frame: %s, publish_frequency: %f", msg.header.frame_id.c_str(), uwb_freq);
    return msg;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "unicorn_roborio_bridge_node");

    ros::NodeHandle nh;

    rear_lidar_pub = nh.advertise<sensor_msgs::LaserScan>("/rearLidar/scan", 10);
    lift_state_pub = nh.advertise<std_msgs::Int32>("/lift/state", 1);
    uwb_pos_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/uwb/pose", 10);

    lift_state = init_lift_msg();
    lidar_scan = init_lidar_msg();
    uwb_pos = init_uwb_msg();

    rio_sub = nh.subscribe("/RIO_publisher", 10, &rio_cb);

    LiftActionServer lift_server(nh, "/lift/lift_action", boost::bind(&execute_lift, _1, &lift_server, nh), false);
    lift_server.start();

    ros::Timer lidar_timer = nh.createTimer(ros::Duration(1.0 / lidar_freq), &lidar_pub_timeout);
    ros::Timer lift_timer = nh.createTimer(ros::Duration(1.0 / lift_freq), &lift_pub_timeout);
    ros::Timer uwb_timer = nh.createTimer(ros::Duration(1.0 / uwb_freq), &uwb_pub_timeout);


    ros::spin();
}
