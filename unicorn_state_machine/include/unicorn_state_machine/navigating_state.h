#ifndef __NAVIGATING_STATE_H_
#define __NAVIGATING_STATE_H_

#include <unicorn_state_machine/state.h>

#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseAction.h>

class NavigatingState : public State {

    public:
        NavigatingState(const geometry_msgs::Pose& pose, int lift_cmd, ros::NodeHandle node);

        virtual State* run() override;

        virtual int stateIdentifier() const override { return 1; }

    private:

        geometry_msgs::Pose goal_;
        int lift_cmd_;

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> movebase_client_;


};

#endif // __NAVIGATING_STATE_H_
