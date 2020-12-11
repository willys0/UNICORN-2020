#ifndef __ERROR_STATE_H_
#define __ERROR_STATE_H_

#include <unicorn_state_machine/state.h>

class ErrorState : public State {

    public:
        ErrorState();

        ErrorState(const std::string& error_msg, ros::NodeHandle node);

        virtual State* run() override;

        virtual int stateIdentifier() const override { return 6; }

        std::string getErrorMsg() const { return error_msg_; }

    protected:
        const std::string error_msg_;

};



#endif // __ERROR_STATE_H_
