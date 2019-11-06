#include "unicorn/state_factory.h"
#include "unicorn/idle_state.h"
#include "unicorn/aligning_state.h"
#include "unicorn/lifting_state.h"
#include "unicorn/navigating_state.h"
#include "unicorn/reversing_state.h"


std::shared_ptr<State> StateFactory::CreateStateInstance(int state)
{
    State * instance = nullptr;

    switch (state)
    {
    case state_enum::IDLE:
        instance = new IDLEState();
        break;

    case state_enum::NAVIGATING:
        instance = new NAVIGATINGState();
        break;
    
    case state_enum::ALIGNING:
        instance = new ALIGNINGState();
        break;

    case state_enum::REVERSING:
        instance = new REVERSINGState();
        break;

    case state_enum::LIFT:
        instance = new LIFTState();
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