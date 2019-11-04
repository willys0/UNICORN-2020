#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>


namespace state_enum
{
	enum
	{
		AUTONOMOUS,
		MANUAL,
		LOADING,
		IDLE,
		ALIGNING,
		EXITING,
		ENTERING,
		LIFT,
		REVERSING
	};
}

typedef struct cmd_struct {
    int state;
    float param1;
    float param2;
    float param3;
} cmd_struct;

class StateMachine
{
    public:
        StateMachine();
        ~StateMachine();
        void initNextState(struct cmd_struct cmd_struct_);
        /**
         * takes the next state and searches the constructor array for the
         * relevant state constructor. Also sends potential move_base
        */
    protected:
    private:
        ros::NodeHandle n_;
        ros::Publisher state_pub_;
        ros::Subscriber cmd_sub_;
        State current_state_;
        void CmdCallback(const std_msgs::String &msg) = 0;
};