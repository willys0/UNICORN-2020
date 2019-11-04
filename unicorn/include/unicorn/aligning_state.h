#include "state.h"

/* State description: 
    The aligning state is instansiated when a goal has been reached, alt has been issued manually. 
    1. Reverse towards bin 
    2. Await message with distance from bin sent by RIO
    3. When Distance has reached 10cm halt
    4. Exit state with next state set to LIFTING
*/

class AligningState : public State
{
public:
    AligningState();
    ~AligningState();
    int run();
protected:
private:
    
};