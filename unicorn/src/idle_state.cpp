#include "unicorn/idle_state.h"

IDLEState::IDLEState()
{
    state_identifier_ = state_enum::IDLE;
    
}

IDLEState::~IDLEState()
{
}

Command IDLEState::run()
{
   
}

//terminate