#include <ros/ros.h>

#include <unicorn_state_machine/state_machine.h>
#include <unicorn_state_machine/goal.h>

#include <geometry_msgs/Pose.h>

#include <math.h>

void parseGoalYaml(ros::NodeHandle& nh, std::vector<struct Goal>& goals) {
    XmlRpc::XmlRpcValue v;

    nh.param("goals", v, v);

    XmlRpc::XmlRpcValue inner;

    double x, y, th;
    int lift;

    struct Goal goal;

    for(int i = 0; i < v.size(); i++) {
        inner = v[i];

        if(inner["x"].valid() && inner["y"].valid() && inner["th"].valid() && inner["lift"].valid()) {
            x    = static_cast<double>(inner["x"]);
            y    = static_cast<double>(inner["y"]);
            th   = static_cast<double>(inner["th"]);
            lift = static_cast<int>(inner["lift"]);

            goal.pose.position.x = x;
            goal.pose.position.y = y;
            goal.pose.position.z = 0.0;

            goal.pose.orientation.w = cos(th/2);
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = sin(th/2);

            goal.lift_cmd = (LiftCommand)lift;

            goals.push_back(goal);

            ROS_INFO("Added goal x: %f, y %f, th %f, lift: %d", x, y, th, lift);
        }
        else {
            ROS_INFO("Invalid format for goal %d", i);
        }
    }

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "unicorn_state_machine_node");


    ros::NodeHandle nh("~");

    std::vector<struct Goal> goals;

    parseGoalYaml(nh, goals);

    for(int i = 0; i < goals.size(); i++) {
        ROS_INFO("Goal %d: (%f, %f), (%f, %f, %f, %f), %d",
                 i,
                 goals[i].pose.position.x, goals[i].pose.position.y,
                 goals[i].pose.orientation.x, goals[i].pose.orientation.y, goals[i].pose.orientation.z, goals[i].pose.orientation.w,
                 goals[i].lift_cmd);
    }

    StateMachine state_machine;

    state_machine.setGoals(goals);
    state_machine.start();

    ros::spin();

    return 0;
}
