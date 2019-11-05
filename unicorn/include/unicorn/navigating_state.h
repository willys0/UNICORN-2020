#include "state.h"

/* State description: 
    The navigating state is instansiated when a move command has been issued. 
    1. Send Goal to move_base
    2. Await goal completion status
        2 alt. if abort signal is issued, cancel goal.
    4. Exit State with next state set to ALIGNING
        4. Exit State with next state set to IDLE if goal was canceled.

*/
class NavigatingState : public State
{
public:
    NavigatingState();
    ~NavigatingState();
    Command run();
protected:
private:
    void cancelGoal();
    int sendGoal(Goal new_goal);
};