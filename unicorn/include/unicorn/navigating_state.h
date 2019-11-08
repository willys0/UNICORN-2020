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
    Command run();
    Command run(ros::NodeHandle node);
protected:
private:
    Goal current_goal_;
    MoveBaseClient move_base_clt_;
    ros::NodeHandle node_;
    ros::Publisher move_base_cancel_pub_;
    void cancelGoal();
    int sendGoal(Goal new_goal);
};