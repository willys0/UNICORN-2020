#ifndef __STATE_H_
#define __STATE_H_

#include <ros/ros.h>

#include <unicorn_state_machine/goal.h>

#ifndef RETURN_ON_ERROR
#define RETURN_ON_ERROR() ({if(error_){return new ErrorState("Forced error", nh_);}})
#endif

class State {
    public:
        /** @brief Default Constructor to initialise the base state class
         *
         */
        State(){}
        /** @brief Constructor taking a node handle as param.
         *
         *	@param node ros::NodeHandle
         */
        State(ros::NodeHandle node) : nh_(node) {}
        /**
           @brief Default de-constructor
        */
        virtual ~State(){};
        /**
         * @brief Pure Virtual function which is called to by the state machine. Intended purpose is to run the state logic and exit upon finished.
         */
        virtual State* run() = 0;

        /**
         * @brief Integer representation of the instiated state
         */
        virtual int stateIdentifier() const = 0;

        static void forceError() { error_ = true; }
        static void removeError() { error_ = false; }

        static void setRepeating(bool repeating) { repeating_ = repeating; }

        static void setGoals(const std::vector<struct Goal>& goals) { goal_it_.setGoals(goals); }

        static void addGoal(const struct Goal& goal) { goal_it_.addGoal(goal); }

        static std::vector<struct Goal> getGoals() { return goal_it_.getGoals();}

        static bool getNextGoal(struct Goal& goal) { return goal_it_.next(goal); }


    protected:
        ros::NodeHandle nh_;

        static bool error_;
        static bool repeating_;

        static GoalIterator goal_it_;

};

#include <unicorn_state_machine/error_state.h>

#endif // __STATE_H_
