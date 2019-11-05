namespace state_identifier
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
} Command;

typedef struct goal_struct
{
    const float x;
    const float y;
    const float yaw;
} Goal;

typedef struct refuse_bin
{
    const float x;
    const float y;
    const float yaw;
} RefuseBin;


