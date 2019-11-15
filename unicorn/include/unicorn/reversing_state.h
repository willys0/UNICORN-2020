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
    /**
     * @brief Class constructor
     * 
     * @param node ros node handle to the state machine node
    */
    REVERSINGState(ros::NodeHandle node);
    /**
     * @brief Class deconstructor
    */
    ~REVERSINGState();
    /**
     * @brief Main logic for the reversing state, returns either a new command or a default IDLE command if an abort message has been issued. 
    */
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
    /**
     * @brief Callback method used to capture and process messages issued by the roboRIO to rear lidar topic. 
     * 
     * @param msg message containing the current averaged distance measured by the rear lidar
    */
    void rearLidarCallback(const std_msgs::Float32 &msg);
    /**
     * @brief Callback method used to capture and process odometry messages
     * 
     * @param msg message containing odometry data
    */
    void odomCallback(const nav_msgs::Odometry &msg);
    /**
     * @brief method which publishes to the cancel goal topic subscribed to by move base. 
    */
    void cancelGoal();

};