#ifndef __AM_UNICORN_INTERFACE_H_
#define __AM_UNICORN_INTERFACE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <am_driver/SensorStatus.h>

/** @brief Main class for automower to unicorn interface node
 *
 *	Flips the velocity command from topic "/unicorn/cmd_vel" and publishes on
 *	"cmd_vel".
 */
class AmUnicornInterface
{
    public:
        AmUnicornInterface();
        void cmdVelCallback(const geometry_msgs::Twist& cmd_vel);
        void amStatusCallback(const am_driver::SensorStatusConstPtr& msg);
        void publishCmd();
    private:
        ros::NodeHandle n_;
        geometry_msgs::Twist unicorn_cmd_vel_;
        ros::Subscriber cmd_vel_sub_;
        ros::Publisher unicorn_cmd_vel_pub_;

        ros::Subscriber hrp_status_sub_;
        ros::Publisher  hrp_mode_pub_;

        ros::Publisher movebase_cancel_pub_;
        ros::Publisher dock_cancel_pub_;
        ros::Publisher lift_cancel_pub_;

        ros::Time last_cmd_vel_time_;

        float max_velocity_delay_;

        bool started_;
};

#endif // __AM_UNICORN_INTERFACE_H_
