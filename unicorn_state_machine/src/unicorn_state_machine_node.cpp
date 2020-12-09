#include <ros/ros.h>

#include <unicorn_state_machine/state_machine.h>
#include <unicorn_state_machine/goal.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_srvs/SetBool.h>
#include <unicorn_state_machine/StampGoal.h>

#include <tf2_ros/transform_listener.h>

#include <math.h>

bool toggleRunningCb(std_srvs::SetBool::Request&  req,
                     std_srvs::SetBool::Response& resp,
                     StateMachine* sm) {

    if(req.data)
        sm->resume();
    else
        sm->pause();

    return true;
}

bool stampGoalCb(unicorn_state_machine::StampGoal::Request&  req,
                 unicorn_state_machine::StampGoal::Response& resp,
                 StateMachine* sm,
                 const std::string& base_frame) {


    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    const std::string world_frame("map");

    try {
        geometry_msgs::TransformStamped robot_transform = tf_buffer.lookupTransform(world_frame, base_frame, ros::Time(0), ros::Duration(1.0));

        geometry_msgs::Pose pose;
        pose.position.x = robot_transform.transform.translation.x;
        pose.position.y = robot_transform.transform.translation.y;
        pose.position.z = 0.0;
        pose.orientation = robot_transform.transform.rotation;

        struct Goal goal { pose, (LiftCommand)req.goal_type };

        sm->addGoal(goal);
        resp.message = std::string("Goal stamped.");
        resp.success = true;
    }
    catch (tf2::TransformException& ex){
        resp.message = std::string("Failed to lookup transform from " + world_frame + " to "  + base_frame);
        resp.success = false;
    }

    return true;
}

void parseGoalYaml(ros::NodeHandle& nh, std::vector<struct Goal>& goals) {
    XmlRpc::XmlRpcValue v;

    nh.param("goals", v, v);

    if(!v.valid())
        return;

    XmlRpc::XmlRpcValue inner;

    double x, y, th;
    int lift;

    struct Goal goal;

    for(int i = v.size()-1; i > -1; i--) {
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

    ros::AsyncSpinner spinner(1);

    StateMachine state_machine;

    std::vector<struct Goal> goals;
    bool publish_poses;
    bool autostart;
    std::string base_frame;

    ros::Publisher pose_pub;
    nh.param("publish_poses", publish_poses, false);
    nh.param("autostart", autostart, false);
    nh.param("robot_base_frame", base_frame, std::string("base_link"));


    ros::ServiceServer run_service = nh.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>("set_running", boost::bind(&toggleRunningCb, _1, _2, &state_machine));

    ros::ServiceServer stamp_service = nh.advertiseService<unicorn_state_machine::StampGoalRequest, unicorn_state_machine::StampGoalResponse>("stamp_goal", boost::bind(&stampGoalCb, _1, _2, &state_machine, base_frame));

    parseGoalYaml(nh, goals);

    for(int i = 0; i < goals.size(); i++) {
        ROS_INFO("Goal %d: (%f, %f), (%f, %f, %f, %f), %d",
                 i,
                 goals[i].pose.position.x, goals[i].pose.position.y,
                 goals[i].pose.orientation.x, goals[i].pose.orientation.y, goals[i].pose.orientation.z, goals[i].pose.orientation.w,
                 goals[i].lift_cmd);
    }


    state_machine.setGoals(goals);

    spinner.start();

    if(autostart)
        state_machine.resume();

    state_machine.start(nh, publish_poses);

    spinner.stop();


    return 0;
}
