#include "unicorn/state.h"

/* State description: 
    The ALIGNING state is instansiated when a goal has been reached, alt has been issued manually. 
    1. Align the robot so it stands perpendicular with the refuse bin
    2. Exit state with new state set to REVERSING
        2 alt. if abort message was recieved exit with new state set to IDLE
*/

class ALIGNINGState : public State
{
public:
    ALIGNINGState(ros::NodeHandle node, RefuseBin bin);
    ~ALIGNINGState();
    Command run();
protected:
private:
    /**
     * @brief method which publishes a new goal to the ROS move base
     * 
     * @param new_goal instance containing the goal information in the form of x, y and yaw coordinates.
    */
    int sendGoal(Goal new_goal);
    /**
     * @brief method which publishes to the cancel goal topic subscribed to by move base. 
    */
    void cancelGoal();
    MoveBaseClient move_base_clt_;
    ros::Publisher move_base_cancel_pub_;
    ros::NodeHandle node_;
    RefuseBin bin_location_;
};