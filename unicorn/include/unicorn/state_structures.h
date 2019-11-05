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

typedef struct cmd_struct
{
    int state;
    float param1;
    float param2;
    float param3;
} cmd_struct;

typedef struct goal_struct
{
    const float x;
    const float y;
    const float yaw;
}Goal;

class RefuseBin
{
public:
    RefuseBin();
    float x;
    float y;
    float yaw;
};

