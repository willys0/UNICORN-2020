#include "unicorn/state.h"

/* State description: 
    The navigating state is instansiated when a move command has been issued. 
    1. Send Goal to move_base
    2. Await goal completion status
        2 alt. if abort signal is issued, cancel goal.
    4. Exit State with next state set to ALIGNING
        4. Exit State with next state set to IDLE if goal was canceled.
*/

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; /**< Client that calls actions from move_base */

class NAVIGATINGState : public State
{
public:
    NAVIGATINGState(float x, float y, float yaw, ros::NodeHandle node);
    ~NAVIGATINGState();
     /**
     * @brief Main logic for the navigating state, returns either a new command or a default IDLE command if an abort message has been issued. 
     */
    Command run();
protected:
private:
    Goal current_goal_;
    MoveBaseClient move_base_clt_;
    ros::NodeHandle node_;
    ros::Publisher move_base_cancel_pub_;
    /**
     * @brief method which publishes to the cancel goal topic subscribed to by move base. 
    */
    void cancelGoal();
    /**
     * @brief method which publishes a new goal to the ROS move base
     * 
     * @param new_goal instance containing the goal information in the form of x, y and yaw coordinates.
    */
    int sendGoal(Goal new_goal);
};