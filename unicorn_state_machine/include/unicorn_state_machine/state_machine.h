#ifndef __STATE_MACHINE_H_
#define __STATE_MACHINE_H_

#include <ros/ros.h>

#include <unicorn_state_machine/goal.h>
#include <unicorn_state_machine/state.h>

class StateMachine {
    public:
        StateMachine(ros::NodeHandle nh, bool repeating = false, bool publish_poses = false);

        void setGoals(const std::vector<struct Goal>& goals);
        void addGoal(struct Goal goal);

        void start();

        void pause();
        void resume();

        void forceStop();
        void setOK();

        bool isRunning() { return !paused_; }

    private:
        ros::NodeHandle nh_;

        std::vector<struct Goal> goals_;

        State* current_state_;

        ros::Publisher state_pub_;
        ros::Publisher pose_pub_;

        bool paused_;
        bool publish_poses_;
};

#endif // __STATE_MACHINE_H_
