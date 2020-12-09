#ifndef __STATE_MACHINE_H_
#define __STATE_MACHINE_H_

#include <ros/ros.h>

#include <unicorn_state_machine/goal.h>

class StateMachine {
    public:
        StateMachine();

        void setGoals(const std::vector<struct Goal>& goals);
        void addGoal(struct Goal goal);

        void start(ros::NodeHandle nh, bool publish_poses = false);

        void pause();
        void resume();

        bool isRunning() { return !paused_; }

    private:
        std::vector<struct Goal> goals_;

        ros::Publisher state_pub_;
        ros::Publisher pose_pub_;

        bool paused_;
        bool publish_poses_;
};

#endif // __STATE_MACHINE_H_
