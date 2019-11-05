#include "unicorn/state_machine.h"

int main(int argc, char** argv) {
    std::string name = "unicorn_statemachine";
	ros::init(argc, argv, name);    
    StateMachine state_machine;
    //state_machine.start();
    return 0;
}

StateMachine::StateMachine() : move_base_clt_("move_base", true)
{
    bool run_global_loc, sim_time;
    std::string odom_topic;
    if (!n_.getParam("use_sim_time", sim_time))
    {
        sim_time = false;
    }
    ROS_INFO("[UNICORN State Machine] sim_time: %i", sim_time);

    if (!n_.getParam("max_angular_vel", max_angular_vel_))
    {
        max_angular_vel_ = 0.5;
    }
    ROS_INFO("[UNICORN State Machine] MAX_ANGULAR_VEL: %f", max_angular_vel_);
    if (!n_.getParam("max_linear_vel", max_linear_vel_))
    {
        max_linear_vel_ = 0.3;
    }
    ROS_INFO("[UNICORN State Machine] MAX_LINEAR_VEL: %f", max_linear_vel_);
    if (!n_.getParam("odometry_topic", odom_topic))
    {
        odom_topic = "odom";
    }
    if (!n_.getParam("frame_id", frame_id_))
    {
        frame_id_ = "base_link";
    }

    if(!n_.getParam("global_local", run_global_loc))
    {
        run_global_loc = false;
    }

    ROS_INFO("[unicorn_statemachine] Robot frame_id: %s", frame_id_.c_str());
    ROS_INFO("[unicorn_statemachine] Listening to %s", odom_topic.c_str());
    state_pub_ = n_.advertise<std_msgs::Int32>("/TX2_unicorn_state", 0);
    move_base_cancel_pub_ = n_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 0);
    amcl_global_clt_ = n_.serviceClient<std_srvs::Empty>("/global_localization");
    cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("/unicorn/cmd_vel", 0);
    odom_sub_ = n_.subscribe(odom_topic.c_str(), 0, &StateMachine::odomCallback, this);
    acc_cmd_srv_ = n_.advertiseService("cmd_charlie", &StateMachine::accGoalServer, this);

    /*Init global localisation if param has been set*/
    if (run_global_loc)
    {
        initGlobalLocalisation();
    }

    /*Create pid controller for x movement*/
    velocity_pid_ = new PidController(0.3, 0.1, 0.0, 0.05);
	velocity_pid_->setLimit(-0.3, 0.3);

    /*Set the current state to be IDLE*/
    current_state_ = new IDLEState();
}

StateMachine::~StateMachine() {}

int StateMachine::start() 
{
    Command new_cmd;
    while (true)
    {
        new_cmd = current_state_->run();
        initNextState(new_cmd);
    }
    return 1;   
}

void StateMachine::initGlobalLocalisation()
{
    std_srvs::Empty srv;
    ros::service::waitForService("/global_localization", 1);

    if (amcl_global_clt_.call(srv))
    {
        ROS_INFO("[am_unicorn_interface]: Initialized amcl global localization");
    }
    else
    {
        ROS_ERROR("[am_unicorn_interface]: Failed to initialize amcl global localization");
    }
}

void StateMachine::initNextState(Command cmd)
{}

void StateMachine::cmdCallback(const std_msgs::String &msg)
{
}

void StateMachine::odomCallback(const nav_msgs::Odometry &msg)
{
    tf::Pose pose;
    tf::poseMsgToTF(msg.pose.pose, pose);
    current_yaw_ = tf::getYaw(pose.getRotation());
    current_vel_ = msg.twist.twist.linear.x;
    return;
}

void StateMachine::updateAndPublishState(int new_state)
{
    std_msgs::Int32 new_state_msg;
    new_state_msg.data = new_state;
    state_pub_.publish(new_state_msg);
    return;
}

// void StateMachine::cancelGoal()
// {
//     actionlib_msgs::GoalID cancel_all;
//     move_base_cancel_pub_.publish(cancel_all);
//     ROS_INFO("[unicorn_statemachine] Canceling move_base goal");
// }

int StateMachine::sendGoal(const float x, const float y, const float yaw)
{
    try
    {
        float check_input = boost::lexical_cast<float>(x);
    }
    catch (boost::bad_lexical_cast &e)
    {
        ROS_INFO("%s", e.what());
        ROS_ERROR("[unicorn_statemachine] x is undefined");
        updateAndPublishState(state_enum::IDLE);
        return -1;
    }
    try
    {
        float check_input = boost::lexical_cast<float>(y);
    }
    catch (boost::bad_lexical_cast &)
    {
        ROS_ERROR("[unicorn_statemachine] y is undefined");
        updateAndPublishState(state_enum::IDLE);
        return -1;
    }
    try
    {
        float check_input = boost::lexical_cast<float>(yaw);
    }
    catch (boost::bad_lexical_cast &)
    {
        ROS_ERROR("[unicorn_statemachine] yaw is undefined");
        updateAndPublishState(state_enum::IDLE);
        return -1;
    }
    while (!move_base_clt_.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

    move_base_clt_.sendGoal(goal);

    return 1;
}

Command StateMachine::parseCmdMsg(std::string cmd_msg)
{
    Command new_cmd;
    // json cmd_object = (cmd_msg.c_str())_json;
    // Command new_cmd{
    //     cmd_object["state"].get<int>(),
    //     cmd_object["param1"].get<float>(),
    //     cmd_object["param2"].get<float>(),
    //     cmd_object["param3"].get<float>()};
    return new_cmd;
}

std::string StateMachine::getStateString(int state_identifier_)
{
    switch (state_identifier_)
    {
    case state_enum::AUTONOMOUS:
        return "AUTONOMOUS";

    case state_enum::MANUAL:
        return "MANUAL";

    case state_enum::LOADING:
        return "LOADING";

    case state_enum::IDLE:
        return "IDLE";

    case state_enum::LIFT:
        return "LIFT";

    case state_enum::REVERSING:
        return "REVERSING";

    case state_enum::ALIGNING:
        return "ALIGNING";

    case state_enum::ENTERING:
        return "ENTERING";

    case state_enum::EXITING:
        return "EXITING";

    default:
        return "INVALID STATE";
    }
}

bool StateMachine::accGoalServer(unicorn::CharlieCmd::Request &req, unicorn::CharlieCmd::Response &res)
{
    if (current_state_->state_identifier_ == state_enum::MANUAL)
    {
        res.response = 0;
        return false;
    }
    if (sendGoal(req.goal.x, req.goal.y, req.goal.theta))
    {
        res.response = 1;
    }
    else
    {
        res.response = 0;
    }
}