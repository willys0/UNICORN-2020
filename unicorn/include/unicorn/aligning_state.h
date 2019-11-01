#include "state.h"

class AligningState : public State
{
public:
    AligningState(std::vector<std::string> topic_names);
    ~AligningState();
    int run();
protected:
private:
    
};