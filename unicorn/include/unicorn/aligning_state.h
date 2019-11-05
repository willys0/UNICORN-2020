#include "state.h"

/* State description: 
    The aligning state is instansiated when a goal has been reached, alt has been issued manually. 
    1. Align the robot so it stands perpendicular with the refuse bin
    2. Exit state with new state set to REVERSING
        2 alt. if abort message was recieved exit with new state set to IDLE
*/

class AligningState : public State
{
public:
    AligningState();
    ~AligningState();
    cmd_struct_ run();
protected:
private:
    
};