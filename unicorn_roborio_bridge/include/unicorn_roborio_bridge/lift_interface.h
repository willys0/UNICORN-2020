#ifndef __LIFT_INTERFACE_H_
#define __LIFT_INTERFACE_H_

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <std_msgs/Int32.h>

#include <unicorn_roborio_bridge/RunLiftAction.h>

class LiftInterface {
    public:
        enum LiftState { IDLE, RUNNING, ERROR };

        LiftInterface(ros::NodeHandle nh);

        void startPickupRoutine();
        void startDropoffRoutine();
        void stopLift();

        LiftState getCurrentState();

        bool isIdle()    { return getCurrentState() == LiftState::IDLE; }
        bool isRunning() { return getCurrentState() == LiftState::RUNNING; }
        bool isErrored() { return getCurrentState() == LiftState::ERROR; }

        void setRioLiftState(int state) { lift_state_.data = state; }


    protected:
        enum RioLiftAction { STOP = 0, PICKUP = 1, DROPOFF = 2 };

        void initLiftMsg();

        void liftPubTimeout(const ros::TimerEvent& e);

        void waitForMsgPublish();

    private:

        ros::NodeHandle nh_;

        ros::Publisher run_lift_pub_;
        ros::Publisher lift_state_pub_;

        ros::Timer lift_pub_timer_;

        std_msgs::Int32 run_msg_;
        std_msgs::Int32 lift_state_;

        float lift_state_pub_freq_;
};

#endif // __LIFT_INTERFACE_H_
