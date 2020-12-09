#include <ros/ros.h>

#include <unicorn_roborio_bridge/RioMasterMsg.h>
#include <std_msgs/Int32.h>

int picking_routine;

void pickingRoutineCallback(const std_msgs::Int32ConstPtr& msg) {
    picking_routine = msg->data;
}

int lift() {
    const static float lift_speed = 0.35;
    static double lift_pos;
    static ros::Time last_time = ros::Time::now();

    ros::Time current_time = ros::Time::now();
    double dt = (ros::Time::now() - last_time).toSec();

    last_time = current_time;

    if(picking_routine == 1) {
        lift_pos += dt * lift_speed;
    }
    else if(picking_routine == 2 || picking_routine == 3) {
        // TOOD: Should not go same direction as pickup routine, but w/e
        lift_pos += dt * lift_speed;
    }
    else {
    }

    if(lift_pos < 0.0 || lift_pos > 6.0) {
        picking_routine = 0;
        lift_pos = 0.0;
    }

    return (int)lift_pos;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "unicorn_roborio_simulator_node");
    ros::NodeHandle n;

    ros::Subscriber lift_sub = n.subscribe("/TX2_unicorn_picking_routine", 1, &pickingRoutineCallback);
    ros::Publisher master_msg_pub = n.advertise<unicorn_roborio_bridge::RioMasterMsg>("/RIO_publisher_master_Message", 1, false);


    unicorn_roborio_bridge::RioMasterMsg msg;

    while(ros::ok()) {
        ros::spinOnce();

        msg.lift_state = lift();

        master_msg_pub.publish(msg);

        ros::Duration(1.0 / 30.0).sleep();
    }

    return 0;
}
