#ifndef __UNDOCK_STATE_H_
#define __UNDOCK_STATE_H_

#include <ros/ros.h>
#include <unicorn_state_machine/state.h>

class UndockState : public State {

    public:
        UndockState(int lift_cmd, ros::NodeHandle node);

        virtual State* run() override;

        virtual int stateIdentifier() const override { return 4; }

    private:
        int lift_cmd_;
};



#endif // __UNDOCK_STATE_H_
