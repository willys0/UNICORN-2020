#include "unicorn/state.h"

/* State description: 
    The reversing state is instansiated when the robot has aligned itself perpendicular to the refuse bin
    1. Reverse towards the bin until the rear lidar distance is less or equal to 10 cm
    2. Exit state with new state set to LIFTING
        2 alt. if abort message was recieved exit with new state set to IDLE
*/

class REVERSINGState : public State
{
public:
    /*Methods*/
    REVERSINGState(ros::NodeHandle node);
    ~REVERSINGState();
    Command run();

private:
    /*Members*/
	geometry_msgs::Twist man_cmd_vel_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher move_base_cancel_pub_;
    ros::Subscriber rear_lidar_sub_;
    float desired_distance_ = 10; //expressed in cm
    bool at_desired_distance_ = false;
    double current_yaw_;
	double current_vel_;
    MoveBaseClient move_base_clt_;
    /*Methods*/
    void rearLidarCallback(const std_msgs::Float32 &msg);
    void odomCallback(const nav_msgs::Odometry &msg);
    void cancelGoal();

};