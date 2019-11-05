#include "state.h"

/* State description: 
    The lifting state is instansiated when the lift is to be activated. 
    1. Send lift command to the RIO topic
    2. Await lift completion signal from RIO
    3. Resverse a small distance to clear from collection site
    4. Exit State with next state set to IDLE
*/

class LiftingState : public State
{
public:
    LiftingState();
    ~LiftingState();
    Command run();

protected:
private:
    void liftCallback(const std_msgs::Int8 &recieveMsg);
    void reverseFromBin();
    ros::Subscriber lift_complete_sub_;
    ros::Publisher lift_init_pub_;
};