#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>

#include <actionlib/client/simple_action_client.h>
#include <unicorn_roborio_bridge/RunLiftAction.h>

typedef actionlib::SimpleActionClient<unicorn_roborio_bridge::RunLiftAction> RunLiftActionClient;

ros::Subscriber joy_sub;
ros::Publisher  state_pub;

void joy_cb(const sensor_msgs::JoyConstPtr& msg, RunLiftActionClient* ac) {
    static std_msgs::Int32 state;

    // L1
    if(msg->buttons[10]) {
        ROS_INFO("L1");

        // Up, Right, Down, Left
        if(msg->buttons[4]) {
            ROS_INFO("Up");
            unicorn_roborio_bridge::RunLiftGoal goal;
            goal.direction = goal.DIRECTION_PICKUP;
            ac->sendGoal(goal);
        }
        else if(msg->buttons[5]) {
            ROS_INFO("Right");
            ac->cancelAllGoals();
            
        }
        else if(msg->buttons[6]) {
            ROS_INFO("Down");
            unicorn_roborio_bridge::RunLiftGoal goal;
            goal.direction = goal.DIRECTION_DROPOFF;
            ac->sendGoal(goal);
        }
        else if(msg->buttons[7]) {
            ROS_INFO("Left");
            ac->cancelAllGoals();
        }

        // Triangle, Circle, Cross, Square
        if(msg->buttons[12]) {
            ROS_INFO("Triangle");
            state.data = 0;    
            state_pub.publish(state);
        }
        else if(msg->buttons[13]) {
            ROS_INFO("Circle");
            state.data = 2;    
            state_pub.publish(state);
        }
        else if(msg->buttons[14]) {
            ROS_INFO("Cross");
            state.data = 3;    
            state_pub.publish(state);
        }
        else if(msg->buttons[15]) {
            ROS_INFO("Square");
            state.data = 4; 
            state_pub.publish(state);
        }
    }
}

int main(int argc, char** argv) {


    ros::init(argc, argv, "unicorn_joy_node");

    ros::NodeHandle nh;

    RunLiftActionClient ac("/lift/lift_action", true);

    joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 5, boost::bind(&joy_cb, _1, &ac));
    state_pub = nh.advertise<std_msgs::Int32>("TX2_unicorn_state", 1, true);

    ROS_INFO("Joy node started");
    ros::spin();

    return 0;
}
