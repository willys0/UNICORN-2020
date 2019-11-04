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

typedef struct cmd_struct
{
    int state;
    float param1;
    float param2;
    float param3;
} cmd_struct;

/*
Description:
The state machine class will handle the overgrapsing logic required by the states so they can
transition and respond to events. Furthermore, it also is responsible for keeping the UNICORNs 
*/

class StateMachine
{
public:
    /*Members*/

    /*Methods*/
    StateMachine();
    ~StateMachine();
    int start();
    void initNextState(struct cmd_struct cmd_struct_);
    /**
         * takes the next state and searches the constructor array for the
         * relevant state constructor. Also sends potential move_base
        */
protected:
private:
    /*Members*/
    ros::NodeHandle n_;
    ros::ServiceClient amcl_global_clt_;
    ros::ServiceServer acc_cmd_srv_;
    ros::Publisher state_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber cmd_sub_;
    ros::Subscriber move_base_cancel_pub_;
    ros::Subscriber odom_sub_;
    MoveBaseClient move_base_clt_;
    State current_state_;

    /*Methods*/
    void cmdCallback(const std_msgs::String &msg);
    cmd_struct parseCmdMsg(std::string cmd_msg);
    void updateAndPublishState(int new_state);
};