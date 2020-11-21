#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <unicorn_docking/DockAction.h>

#include <unicorn_docking/docking_controller.h>

#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<unicorn_docking::DockAction> DockActionServer;

geometry_msgs::Point desired_offset;

void execute_action(const unicorn_docking::DockGoalConstPtr& goal, DockActionServer* as, DockingController* controller, ros::NodeHandle nh) {
    ros::Publisher vel_pub;
    geometry_msgs::Twist move_msg;

    unicorn_docking::DockFeedback fbk;

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


    controller->reset();
    controller->setDesiredOffset(desired_offset);
    controller->setState(DockingController::DockState::DOCKING);

    ROS_INFO("[Docking Controller] Executing dock action...");

    while(true) {
        if(as->isPreemptRequested()) {
            unicorn_docking::DockResult rslt;
            // TODO: Add data to results
            as->setPreempted(rslt);

            ROS_INFO("[Docking Controller] Dock action preempted.");
            break;
        }
        if(controller->computeVelocity(move_msg)) {

            controller->setState(DockingController::DockState::IDLE);

            unicorn_docking::DockResult rslt;
            // TODO: Add data to results
            as->setSucceeded(rslt);

            ROS_INFO("[Docking Controller] Dock action finished!");
            break;
        }

        // TODO: Add data to feedback
        as->publishFeedback(fbk);

        vel_pub.publish(move_msg);
        ros::Duration(1.0/100.0).sleep();
    }
    move_msg.linear.x = 0.0;
    move_msg.angular.z = 0.0;
    vel_pub.publish(move_msg);
    // TODO: Handle errors in some nice way

    controller->setState(DockingController::DockState::IDLE);
    ros::Duration(0.1).sleep();
    vel_pub.shutdown();

}

int main(int argc, char **argv) {

    // Init ros node
    ros::init(argc, argv, "unicorn_docking_node");

    ros::NodeHandle nh("~");

    nh.param("offset/x",desired_offset.x, 0.2);
    nh.param("offset/y",desired_offset.y, 0.0);
    nh.param("offset/th",desired_offset.z, 0.0);
    ROS_INFO("Setting dock offset to x: %.2f, y: %.2f, th: %.2f", desired_offset.x, desired_offset.y, desired_offset.z);

    DockingController controller;

    DockActionServer server(nh, "dock_action", boost::bind(&execute_action, _1, &server, &controller, nh), false);

    server.start();

    ros::spin();
    
    return 0;
}
