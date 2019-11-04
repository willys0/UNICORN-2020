#include <idle_state.h>

IDLEState::IDLEState()
{
}

IDLEState::~IDLEState()
{
}

cmd_struct_ IDLEState::Run()
{
    ros::Rate period(50);
    cmd_struct_ cmd;
    //display user interface
    // displayUserInteface();
    while (ros::ok())
    {
        //check if command has been received
        if (!cmd_msg_str_.empty())
        {
            //parse command string to command structure
            cmd = parseMsg(cmd_msg_str_);
            cmd_msg_str_.clear();
            break;
        }
        // int getUserInput();
        ros::spinOnce();
        period.sleep();
    }
    //return new command
    return cmd;
}

void IDLEState::CmdCallback(const std_msgs::String &msg)
{
    cmd_msg_str_ = msg.data;
}

cmd_struct IDLEState::parseMsg(std::string &msg)
{
    json cmd_object = msg_json;
    cmd_struct new_cmd{
        cmd_object["state"].get<int>(),
        cmd_object["param1"].get<float>(),
        cmd_object["param2"].get<float>(),
        cmd_object["param3"].get<float>()};
    return new_cmd;
}
//terminate