#include "state.h"

class IDLEState : public State
{
public:
    IDLEState();

    /*
        
    */

    ~IDLEState();
    int run();
    void CmdCallback(std_msgs::string &cmd);
protected:
private:
    void displayUserInterface();
    int getUserInput();
    void processUserInput(int choice);

};