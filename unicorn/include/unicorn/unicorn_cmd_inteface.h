#include <ros/ros.h>
#include "state_structures.h"

/*
    Command interface: Runs as a seperate node connected to the same network as 
    the robot. 
 */
class UNICORNCmdInterface
{
public:
    UNICORNCmdInterface();
    ~UNICORNCmdInterface();
    void displayUserInterface();
    int getUserInput();
    Command processUserInput(int choice);
    void sendUNICORNCommand(Command cmd);

protected:
private:
    ros::Publisher unicorn_cmd_sub_;
}