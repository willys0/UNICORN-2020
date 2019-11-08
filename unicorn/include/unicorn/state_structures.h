#ifndef STATE_STRUCTURES_H
#define STATE_STRUCTURES_H
namespace state_enum
{
enum
{
    NAVIGATING,
    MANUAL,
    LOADING,
    IDLE,
    ALIGNING,
    EXITING,
    ENTERING,
    LIFT,
    REVERSING
};
}

typedef struct command
{
    command() : state(-1), param1(-1.0), param2(-1.0), param3(-1.0) {}
    int state;
    float param1;
    float param2;
    float param3;
} Command;

typedef struct goal
{
    goal() : x(-1), y(-1), yaw(-1) {}
    float x;
    float y;
    float yaw;
} Goal;

typedef struct refuseBin
{
    refuseBin() : x(-1), y(-1), yaw(-1) {}
    float x;
    float y;
    float yaw;
} RefuseBin;

#endif // !ST
