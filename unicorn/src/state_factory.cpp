#include "unicorn/state_factory.h"
#include "unicorn/idle_state.h"
#include "unicorn/aligning_state.h"
#include "unicorn/lifting_state.h"
#include "unicorn/navigating_state.h"
#include "unicorn/reversing_state.h"

std::unique_ptr<State> StateFactory::CreateStateInstance(Command cmd, ros::NodeHandle node, RefuseBin bin)
{
    State *instance = nullptr;

    switch (cmd.state)
    {
    case STATE_IDLE:
        instance = new IDLEState();
        break;

    case STATE_NAVIGATING:
        instance = new NAVIGATINGState(cmd.param1, cmd.param2, cmd.param3, node);
        break;

    case STATE_ALIGNING:
        instance = new ALIGNINGState(node, bin);
        break;

    case STATE_REVERSING:
        instance = new REVERSINGState(node);
        break;

    case STATE_LIFT:
        instance = new LIFTState(node);
        break;

    default:
        break;
    }
    if (instance != nullptr)
    {
        return std::unique_ptr<State>(instance);
    }
    return nullptr;
}