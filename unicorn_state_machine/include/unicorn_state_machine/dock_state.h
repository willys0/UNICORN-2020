#ifndef __DOCK_STATE_H_
#define __DOCK_STATE_H_

#include <ros/ros.h>
#include <unicorn_state_machine/state.h>

class DockState : public State {

    public:
        DockState(int lift_cmd, ros::NodeHandle node);

        virtual State* run() override;

        virtual int stateIdentifier() const override { return 2; }

    private:
        int lift_cmd_;
};

#endif // __DOCK_STATE_H_
