#include "unicorn/state_factory.h"
#include "unicorn/idle_state.h"
#include "unicorn/aligning_state.h"
#include "unicorn/lifting_state.h"
#include "unicorn/navigating_state.h"
#include "unicorn/reversing_state.h"


std::shared_ptr<State> StateFactory::CreateStateInstance(Command cmd, ros::NodeHandle node, RefuseBin bin)
{
    State * instance = nullptr;

    switch (cmd.state)
    {
    case state_enum::IDLE:
        instance = new IDLEState();
        break;

    case state_enum::NAVIGATING:
        instance = new NAVIGATINGState(cmd.param1, cmd.param2, cmd.param3, node);
        break;
    
    case state_enum::ALIGNING:
        instance = new ALIGNINGState(node, bin);
        break;

    case state_enum::REVERSING:
        instance = new REVERSINGState(node);
        break;

    case state_enum::LIFT:
        instance = new LIFTState(node);
        break;

    default:
        break;
    }
    if(instance != nullptr)
    {
        return std::shared_ptr<State>(instance);
    }
    return nullptr;    
}