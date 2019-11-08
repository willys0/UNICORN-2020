#ifndef STATE_FACTORY_H
#define STATE_FACTORY_H
#include "unicorn/state.h"

#include <memory>
#include <string>

class StateFactory
{
private:
    /* data */
public:
    static std::shared_ptr<State> CreateStateInstance(Command cmd, ros::NodeHandle node, RefuseBin bin);
};
#endif // !STATE_FAC
