#ifndef STATE_FACTORY_H
#define STATE_FACTORY_H
#include "unicorn/state.h"

#include <memory>
#include <string>

class StateFactory
{
private:
public:
    /** @brief Creates a new state instance based on the cmd param. The node handle and the refuse bin are used in specific cases.
	*
	* 
	* @param cmd the most recently issued command
	* @param node ros node handle to the state machine node
    * @param bin location information about the refuse bin, expressed in x,y and yaw coordinates.
	*/
    static std::unique_ptr<State> CreateStateInstance(Command cmd, ros::NodeHandle node, RefuseBin bin);
};
#endif // !STATE_FAC
