#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>

ros::Subscriber joy_sub;
ros::Publisher  state_pub;

void joy_cb(const sensor_msgs::JoyConstPtr& msg) {
    static std_msgs::Int32 state;

    // L1
    if(msg->buttons[10]) {

        // Triangle, Circle, Cross, Square
        if(msg->buttons[12]) {
            state.data = 0;    
            state_pub.publish(state);
        }
        else if(msg->buttons[13]) {
            state.data = 2;    
            state_pub.publish(state);
        }
        else if(msg->buttons[14]) {
            state.data = 3;    
            state_pub.publish(state);
        }
        else if(msg->buttons[15]) {
            state.data = 4; 
            state_pub.publish(state);
        }
    }
}

int main(int argc, char** argv) {


    ros::init(argc, argv, "unicorn_joy_node");

    ros::NodeHandle nh;

    joy_sub = nh.subscribe("joy", 5, &joy_cb);
    state_pub = nh.advertise<std_msgs::Int32>("TX2_unicorn_state", 1, true);

    ros::spin();

    return 0;
}