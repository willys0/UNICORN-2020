#include "unicorn/state.h"
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
    /**
     * @brief Default constructor 
    */
    IDLEState();
    /**
     * @brief Default de-constructor
    */
    ~IDLEState();
     /**
     * @brief Main logic for the IDLE state. Waits for a new command to be issued when transitions to the relevant state. 
     */
    Command run();
    /**
     * @brief Setter-method to set a new command for a state. Is used to signal the currently executing state that a new command has been issued. 
     * 
     * @param new_cmd New command which has been published by the command node.
    */
    Command setNewCmd(Command new_cmd);
protected:
private:
};