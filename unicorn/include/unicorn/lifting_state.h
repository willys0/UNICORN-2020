#include "state.h"

class LiftingState : public State
{
public:
    LiftingState(std::vector<std::string> topic_names);
    ~LiftingState();
    int run();

protected:
private:
};