#include <ros/ros.h>

#include <unicorn_state_machine/state_machine.h>

void parse_goal_yaml() {
    ros::NodeHandle nh("~");
    XmlRpc::XmlRpcValue v;

    nh.param("goals", v, v);

    for(int i = 0; i < v.size(); i++) {
        ROS_INFO("x: %f, y %f, th %f, lift: %d");
    }
}

void goal_poller() {

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "unicorn_state_machine_node");



    StateMachine state_machine;

    state_machine.start();

    ros::spin();

    return 0;
}
