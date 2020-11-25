#ifndef __STATE_MACHINE_H_
#define __STATE_MACHINE_H_

#include <unicorn_state_machine/goal.h>

class StateMachine {
    public:
        StateMachine();

        void setGoals(const std::vector<struct Goal>& goals);
        void start();

    private:
        std::vector<struct Goal> goals_;

};

#endif // __STATE_MACHINE_H_
