#include <ros/ros.h>

#include <unicorn_state_machine/state_machine.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "unicorn_state_machine_node");

    StateMachine state_machine;

    state_machine.start();

    ros::spin();

    return 0;
}
