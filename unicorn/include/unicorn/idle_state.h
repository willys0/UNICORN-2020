#include "state.h"
/* State description: 
    The idle state is instansiated as the first state on boot-up, or if the current state has been aborted. 
    1. Print the user interface to console
    2. Await new command on ROS Topic Web_unicorn_cmd
        2 alt. if a key has been sent in the console take it as command
    3. Exit with next state set to what's specified in new command
*/
class IDLEState : public State
{
public:
    IDLEState();
    ~IDLEState();
    Command run();

protected:
private:
};