#include "unicorn/state_machine.h"

int main(int argc, char** argv) 
{
	ros::init(argc, argv, "TX2_unicornStatemachine");    
    StateMachine state_machine;
    int exit_code = state_machine.start();
    ROS_INFO("[UNICORN] State machine exited with: %d", exit_code);
    return 0;
}

StateMachine::StateMachine() : move_base_clt_("move_base", true)
{
    bool run_global_loc, sim_time;
    std::string odom_topic;
    /*Extract params specified in the launch file*/
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
    ROS_INFO("[UNICORN State Machine] Robot frame_id: %s", frame_id_.c_str());
    ROS_INFO("[UNICORN State Machine] Listening to %s", odom_topic.c_str());

    /*Setup ROS subscribers and publishers*/
    state_pub_ = n_.advertise<std_msgs::Int32>("/TX2_unicorn_state", 0);
    amcl_global_clt_ = n_.serviceClient<std_srvs::Empty>("/global_localization");
    cmd_vel_pub_ = n_.advertise<geometry_msgs::Twist>("/unicorn/cmd_vel", 0);
    odom_sub_ = n_.subscribe(odom_topic.c_str(), 0, &StateMachine::odomCallback, this);
    acc_cmd_srv_ = n_.advertiseService("cmd_charlie", &StateMachine::accGoalServer, this);
    cmd_sub_ = n_.subscribe(command_topic_, 0, &StateMachine::cmdCallback, this);

    /*Init global localisation if param is set*/
    if (run_global_loc)
    {
        initGlobalLocalisation();
    }

    /*Create pid controller for x movement*/
    velocity_pid_ = new PidController(0.3, 0.1, 0.0, 0.05);
	velocity_pid_->setLimit(-0.3, 0.3);

    /*Refuse bin location*/
    refuse_bin_pose_.x = 0;
    refuse_bin_pose_.y = 0;
    refuse_bin_pose_.yaw = 0;

    /*Set the current state to be IDLE*/
    Command initial_cmd;
    initial_cmd.state = STATE_REVERSING;
    current_state_ = StateFactory::CreateStateInstance(initial_cmd, n_, refuse_bin_pose_);
}

StateMachine::~StateMachine() {}

int StateMachine::start() 
{
    Command new_cmd;
    ROS_INFO("[UNICORN State Machine] Starting state machine...");
    while (ros::ok())
    {
        updateAndPublishState(current_state_->state_identifier_);
        new_cmd = current_state_->run();
        current_state_.reset();
        current_state_ = StateFactory::CreateStateInstance(new_cmd, n_, refuse_bin_pose_);
    }
    return 1;   
}

void StateMachine::initGlobalLocalisation()
{
    std_srvs::Empty srv;
    ros::service::waitForService("/global_localization", 1);

    if (amcl_global_clt_.call(srv))
    {
        ROS_INFO("[AM Unicorn Interface]: Initialized amcl global localization");
    }
    else
    {
        ROS_ERROR("[AM Unicorn Interface]: Failed to initialize amcl global localization");
    }
}

void StateMachine::cmdCallback(const unicorn::command &msg)
{
    ROS_INFO("[Unicorn State Machine]: New command has been received");
    Command new_cmd;

    new_cmd.state = msg.state;
    new_cmd.param1 = msg.param1;
    new_cmd.param2 = msg.param2;
    new_cmd.param3 = msg.param3;
    current_state_->setNewCmd(new_cmd);
    return;
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
    ROS_INFO("[UNICORN State Machine] Updating state to: %s", getStateString().c_str());
    std_msgs::Int32 new_state_msg;
    new_state_msg.data = new_state;
    state_pub_.publish(new_state_msg);
    return;
}

int StateMachine::sendGoal(const float x, const float y, const float yaw)
{
    int attempts = 0;
    try
    {
        float check_input = boost::lexical_cast<float>(x);
    }
    catch (boost::bad_lexical_cast &e)
    {
        ROS_INFO("%s", e.what());
        ROS_ERROR("[UNICORN State Machine] x is undefined");
        updateAndPublishState(STATE_IDLE);
        return -1;
    }
    try
    {
        float check_input = boost::lexical_cast<float>(y);
    }
    catch (boost::bad_lexical_cast &)
    {
        ROS_ERROR("[UNICORN State Machine] y is undefined");
        updateAndPublishState(STATE_IDLE);
        return -1;
    }
    try
    {
        float check_input = boost::lexical_cast<float>(yaw);
    }
    catch (boost::bad_lexical_cast &)
    {
        ROS_ERROR("[UNICORN State Machine] yaw is undefined");
        updateAndPublishState(STATE_IDLE);

        return -1;
    }
    while (!move_base_clt_.waitForServer(ros::Duration(5.0)) && (attempts < 4))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
        attempts++;
    }
    if(attempts == 3)
    {
        ROS_WARN("Failed to contact move base server after four attempts, goal not published!");
        return -1;
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

std::string StateMachine::getStateString()
{
    switch (current_state_->state_identifier_)
    {
    case STATE_NAVIGATING:
        return "NAVIGATING";

    case STATE_IDLE:
        return "IDLE";

    case STATE_LIFT:
        return "LIFT";

    case STATE_REVERSING:
        return "REVERSING";

    case STATE_ALIGNING:
        return "ALIGNING";

    default:
        return "INVALID STATE";
    }
}

bool StateMachine::accGoalServer(unicorn::CharlieCmd::Request &req, unicorn::CharlieCmd::Response &res)
{
    if (current_state_->state_identifier_ == STATE_IDLE)
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
