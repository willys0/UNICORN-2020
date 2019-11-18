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
    /**
     * @brief Default constructor
     * 
     * @param node ROS node handle pointing to the state machine node.
     */
    LIFTState(ros::NodeHandle node);

    /**
     * @brief Default de-constructor
    */
    ~LIFTState();
    /**
     * @brief Main logic for the lift state, returns either a new command or a default IDLE command if an abort message has been issued. 
     */
    Command run();
protected:
private:
    /*Methods*/
    /**
     * @brief Callback method used to capture messages sent by the roboRIO when the lift state machine is complete. 
     * 
     * @param recieveMsg message containing the current state of the lift state machine.
     */
    void liftCallback(const std_msgs::Int32 &recieveMsg);
    /**
     * @brief reverses from the bin for a small moment 
     */
    void reverseFromBin();
    /**
     * @brief method which publishes to the cancel goal topic subscribed to by move base. 
     */
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
