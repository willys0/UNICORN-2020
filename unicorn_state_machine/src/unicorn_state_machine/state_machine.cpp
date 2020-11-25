#include <unicorn_state_machine/state_machine.h>

#include <unicorn_state_machine/idle_state.h>

#include <std_msgs/Int32.h>

StateMachine::StateMachine() {
   
}

void StateMachine::setGoals(const std::vector<struct Goal> &goals) {
    goals_ = goals;
}

void StateMachine::start() {
    ros::NodeHandle nh;

    State::setGoals(goals_);

    State* currentState = new IdleState(nh);
    State* newState;

    std_msgs::Int32 state_msg;

    ros::Publisher state_pub = nh.advertise<std_msgs::Int32>("current_state", 1, true);

    while(ros::ok()) {
        state_msg.data = currentState->stateIdentifier();
        state_pub.publish(state_msg);

        newState = currentState->run();
        delete currentState;
        currentState = newState;
    }
}
