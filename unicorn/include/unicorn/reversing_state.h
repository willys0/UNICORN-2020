#include "state.h"

/* State description: 
    The reversing state is instansiated when the robot has aligned itself perpendicular to the refuse bin
    1. Reverse towards the bin until the rear lidar distance is less or equal to 10 cm
    2. Exit state with new state set to LIFTING
        2 alt. if abort message was recieved exit with new state set to IDLE
*/

class ReversingState : public State
{
public:
    /*Methods*/
    ReversingState(const float dist_to_bin);
    ~ReversingState();
    Command run();

private:
    /*Members*/
    ros::Subscriber rear_lidar_sub_;
    const float desired_distance_;
    /*Methods*/
    void rearLidarCallback(const std_msgs::Float32 &msg);
};