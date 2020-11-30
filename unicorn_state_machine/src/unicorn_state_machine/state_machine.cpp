#include <unicorn_state_machine/state_machine.h>

#include <unicorn_state_machine/idle_state.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseArray.h>

StateMachine::StateMachine() {
   
}

void StateMachine::setGoals(const std::vector<struct Goal> &goals) {
    goals_ = goals;
}

void StateMachine::start(ros::NodeHandle nh, bool publish_poses) {

    State::setGoals(goals_);

    State* currentState = new IdleState(nh);
    State* newState;

    std_msgs::Int32 state_msg;

    ros::Publisher state_pub = nh.advertise<std_msgs::Int32>("current_state", 1, true);
    ros::Publisher pose_pub;

    if(publish_poses) {
        pose_pub = nh.advertise<geometry_msgs::PoseArray>("goal_poses", 1, true);
    }

    while(ros::ok()) {
        state_msg.data = currentState->stateIdentifier();
        state_pub.publish(state_msg);

        if(publish_poses) {
            geometry_msgs::PoseArray pa;
            for(struct Goal& g : currentState->getGoals()) {
                pa.poses.push_back(g.pose);
            }
            pa.header.stamp = ros::Time::now();
            pa.header.frame_id = "map";

            pose_pub.publish(pa);
        }

        newState = currentState->run();
        delete currentState;
        currentState = newState;
    }
}
