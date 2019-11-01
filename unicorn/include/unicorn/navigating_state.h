#include "state.h"

class NavigatingState : public State
{
public:
    NavigatingState(std::vector<std::string> topic_names);
    ~NavigatingState();
    int run();

protected:
private:
};