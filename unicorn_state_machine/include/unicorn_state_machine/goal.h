#ifndef __GOAL_H_
#define __GOAL_H_

#include <geometry_msgs/Pose.h>

enum LiftCommand { None = 0, Pickup = 1, Dropoff = 2};

struct Goal {
    geometry_msgs::Pose pose;
    LiftCommand lift_cmd;
};

class GoalIterator {
    public:
        GoalIterator(){i_ = 0;}

        bool next(struct Goal& g) {
            if(i_ >= goals_.size())
                return false;
            g = goals_[i_++];
            return true;
        }
        void reset() { i_ = 0; }
        void setGoals(const std::vector<struct Goal>& goals) { goals_ = goals;}
        void addGoal(const struct Goal& goal) { goals_.push_back(goal); }
        std::vector<struct Goal> getGoals() { return goals_; }

    private:
        int i_;
        std::vector<struct Goal> goals_;

};



#endif // __GOAL_H_
