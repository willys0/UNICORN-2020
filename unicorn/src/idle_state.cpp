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

//terminate