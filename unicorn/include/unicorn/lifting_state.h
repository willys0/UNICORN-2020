#include "unicorn/state.h"

/* State description: 
    The lifting state is instansiated when the lift is to be activated. 
    1. Send lift command to the RIO topic
    2. Await lift completion signal from RIO
    3. Resverse a small distance to clear from collection site
    4. Exit State with next state set to IDLE
*/

class LIFTState : public State
{
public:
    LIFTState(ros::NodeHandle node);
    ~LIFTState();
    Command run();
protected:
private:
    /*Methods*/
    void liftCallback(const std_msgs::Bool &recieveMsg);
    void reverseFromBin();
    void cancelGoal();
    /*Members*/
    ros::Subscriber lift_complete_sub_;
    ros::Publisher lift_init_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher move_base_cancel_pub_;
    MoveBaseClient move_base_clt_;
	geometry_msgs::Twist man_cmd_vel_;
    bool lift_complete_ = false;
};