#ifndef STATE_STRUCTURES_H
#define STATE_STRUCTURES_H
namespace state_enum
{
enum
{
    AUTONOMOUS,
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

typedef struct Command
{
    int state;
    float param1;
    float param2;
    float param3;
} Command;

typedef struct Goal
{
    float x;
    float y;
    float yaw;
} Goal;

typedef struct RefuseBin
{
    float x;
    float y;
    float yaw;
} RefuseBin;

#endif // !ST
