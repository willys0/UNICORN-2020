#ifndef __LIFT_STATE_H_
#define __LIFT_STATE_H_

#include <ros/ros.h>
#include <unicorn_state_machine/state.h>

class LiftState : public State {

    public:
        LiftState(int lift_cmd, ros::NodeHandle node);

        virtual State* run() override;

        virtual int stateIdentifier() const override { return 3; }

    private:
        int lift_cmd_;
};


#endif // __LIFT_STATE_H_
