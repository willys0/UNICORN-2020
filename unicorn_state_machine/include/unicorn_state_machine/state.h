#ifndef __STATE_H_
#define __STATE_H_

#include <ros/ros.h>

#include <unicorn_state_machine/goal.h>

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

        static void setGoals(std::vector<struct Goal> goals) { goals_ = goals; }

        static std::vector<struct Goal> getGoals() { return goals_;}


    protected:
        ros::NodeHandle nh_;

        static std::vector<struct Goal> goals_;

};


#endif // __STATE_H_
