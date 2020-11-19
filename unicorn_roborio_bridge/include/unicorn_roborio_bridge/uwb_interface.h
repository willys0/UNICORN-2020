#ifndef __UWB_INTERFACE_H_
#define __UWB_INTERFACE_H_

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class UwbInterface {
    public:
        UwbInterface(ros::NodeHandle nh);

        void setUwbPosition(const geometry_msgs::Point& pos);

    protected:
        void initUwbMsg();

        void uwbPubTimeout(const ros::TimerEvent& e);

    private:
        ros::NodeHandle nh_;

        ros::Publisher uwb_pub_;

        ros::Timer uwb_pub_timer_;

        geometry_msgs::PoseWithCovarianceStamped uwb_pose_msg_;

        float uwb_pub_freq_;

};

#endif // __UWB_INTERFACE_H_
