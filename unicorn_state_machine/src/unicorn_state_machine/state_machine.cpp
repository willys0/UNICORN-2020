#include <unicorn_state_machine/state_machine.h>

#include <unicorn_state_machine/idle_state.h>

#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseArray.h>

StateMachine::StateMachine() {
    paused_ = true;
   
}

void StateMachine::setGoals(const std::vector<struct Goal> &goals) {
    goals_ = goals;

    State::setGoals(goals_);
}

void StateMachine::addGoal(struct Goal goal) {
    State::addGoal(goal);

    ROS_INFO("Added goal (%f, %f), (%f, %f, %f %f), %d",
             goal.pose.position.x, goal.pose.position.y,
             goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w,
             goal.lift_cmd);

    if(publish_poses_) {
        geometry_msgs::PoseArray pa;
        for(struct Goal& g : State::getGoals()) {
            pa.poses.push_back(g.pose);
        }
        pa.header.stamp = ros::Time::now();
        pa.header.frame_id = "map";

        pose_pub_.publish(pa);
    }
}

void StateMachine::start(ros::NodeHandle nh, bool publish_poses) {

    publish_poses_ = publish_poses;

    State* currentState = new IdleState(nh);
    State* newState;

    std_msgs::Int32 state_msg;

    state_pub_ = nh.advertise<std_msgs::Int32>("current_state", 1, true);

    if(publish_poses) {
        pose_pub_ = nh.advertise<geometry_msgs::PoseArray>("goal_poses", 1, true);
    }

    while(ros::ok()) {
        state_msg.data = currentState->stateIdentifier();
        state_pub_.publish(state_msg);

        if(publish_poses_) {
            geometry_msgs::PoseArray pa;
            for(struct Goal& g : currentState->getGoals()) {
                pa.poses.push_back(g.pose);
            }
            pa.header.stamp = ros::Time::now();
            pa.header.frame_id = "map";

            pose_pub_.publish(pa);
        }

        while(paused_) {
            if(!ros::ok()) {
                return;
            }

            ros::Duration(0.5).sleep();
        }

        newState = currentState->run();
        delete currentState;
        currentState = newState;
    }
}

void StateMachine::pause() {
    paused_ = true;
}

void StateMachine::resume() {
    paused_ = false;
}
