#include <unicorn_state_machine/state_machine.h>

#include <unicorn_state_machine/idle_state.h>

StateMachine::StateMachine() {
   
}

void StateMachine::start() {
    ros::NodeHandle nh;

    State* currentState = new IdleState(nh);
    State* newState;

    while(ros::ok()) {
        newState = currentState->run();
        delete currentState;
        currentState = newState;
    }
}
