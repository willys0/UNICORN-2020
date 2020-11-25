#ifndef __IDLE_STATE_H_
#define __IDLE_STATE_H_

#include <unicorn_state_machine/state.h>

class IdleState : public State {

    public:
        IdleState();

        IdleState(ros::NodeHandle node);

        virtual State* run() override;

        virtual int stateIdentifier() const override { return 0; }
};

#endif // __IDLE_STATE_H_
