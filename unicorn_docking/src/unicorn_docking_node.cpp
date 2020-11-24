#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <unicorn_docking/DockAction.h>

#include <unicorn_docking/docking_controller.h>

#include <actionlib/server/simple_action_server.h>

#include <dynamic_reconfigure/server.h>
#include <unicorn_docking/DockingControllerConfig.h>


typedef actionlib::SimpleActionServer<unicorn_docking::DockAction> DockActionServer;

enum DockStatus { RUNNING, SUCCESS, FAILED };

double retry_offset;

geometry_msgs::Point desired_offset;

int max_retries;
int max_error_times;
double thresh_x;
double thresh_y;
double thresh_th;


void dynamicReconfigCallback(unicorn_docking::DockingControllerConfig& config, uint32_t level) {
    thresh_x = config.x_error_thresh;
    thresh_y = config.y_error_thresh;
    thresh_th = config.th_error_thresh;
}

DockStatus dock(DockingController* controller, DockActionServer* as, geometry_msgs::Point thresholds, geometry_msgs::Twist& out_velocity) {
    controller->computeVelocity(out_velocity);

    if(fabs(controller->xError()) <= thresholds.x) {
        if(fabs(controller->yError()) > thresholds.y || fabs(controller->thError()) > thresholds.z) {
            return DockStatus::FAILED;
        }
        else {
            return DockStatus::SUCCESS;
        }
    }

    return DockStatus::RUNNING;
}

void execute_action(const unicorn_docking::DockGoalConstPtr& goal, DockActionServer* as, DockingController* controller, ros::NodeHandle nh) {
    ros::Publisher vel_pub;
    geometry_msgs::Twist move_msg;

    unicorn_docking::DockFeedback fbk;
    unicorn_docking::DockResult rslt;

    int remaining_retries = 10;

    DockStatus dock_status;

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


    geometry_msgs::Point offset = desired_offset;
    geometry_msgs::Point thresh;
    thresh.x = thresh_x;
    thresh.y = thresh_y;
    thresh.z = thresh_th;

    bool docking = true;
    int error_times = 0;

    controller->reset();
    controller->setDesiredOffset(desired_offset);
    controller->setState(DockingController::DockState::DOCKING);

    ROS_INFO("[Docking Controller] Executing dock action...");

    while(true) {
        if(as->isPreemptRequested()) {
            rslt.forward_error = controller->xError();
            rslt.lateral_error = controller->yError();
            rslt.angular_error = controller->thError();
            rslt.success = false;

            as->setPreempted(rslt);

            ROS_INFO("[Docking Controller] Dock action preempted with errors x: %f, y: %f, th: %f!", controller->xError(), controller->yError(), controller->thError());
            break;
        }

        dock_status = dock(controller, as, thresh, move_msg);

        if(dock_status == DockStatus::SUCCESS) {
            controller->setState(DockingController::DockState::IDLE);

            rslt.forward_error = controller->xError();
            rslt.lateral_error = controller->yError();
            rslt.angular_error = controller->thError();
            rslt.success = true;
            as->setSucceeded(rslt);

            ROS_INFO("[Docking Controller] Dock action finished with errors x: %f, y: %f, th: %f!", controller->xError(), controller->yError(), controller->thError());
            break;
        }
        else if(dock_status == DockStatus::FAILED && error_times++ > max_error_times) {
            if(remaining_retries > 0) {
                if(docking) {
                    offset.x = retry_offset;
                    thresh.x = thresh_x * 4;
                }
                else {
                    offset.x = desired_offset.x;
                    thresh.x = thresh_x;
                    remaining_retries--;

                }

                controller->setDesiredOffset(offset);
                docking = !docking;

                error_times = 0;
            }
            else {
                rslt.forward_error = controller->xError();
                rslt.lateral_error = controller->yError();
                rslt.angular_error = controller->thError();
                rslt.success = false;
                as->setPreempted(rslt);

                ROS_INFO("[Docking Controller] Dock action failed after %d retries with errors x: %f, y: %f, th: %f!", max_retries, controller->xError(), controller->yError(), controller->thError());

                break;
            }
        }

        vel_pub.publish(move_msg);

        fbk.forward_error = controller->xError();
        fbk.lateral_error = controller->yError();
        fbk.angular_error = controller->thError();
        as->publishFeedback(fbk);

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

    int n_pitch_avg;
    // load settings
    nh.param("nr_for_pitch_average", n_pitch_avg , 15);
    nh.param("retry_error_times", max_error_times, 50);
    nh.param("max_retries", max_retries, 3);
    nh.param("thresh_x", thresh_x, 0.01);
    nh.param("thresh_y", thresh_y, 0.01);
    nh.param("thresh_th", thresh_th, 0.01);

    nh.param("retry_offset", retry_offset, 0.6);
    nh.param("offset/x",desired_offset.x, 0.2);
    nh.param("offset/y",desired_offset.y, 0.0);
    nh.param("offset/th",desired_offset.z, 0.0);
    ROS_INFO("Setting dock offset to x: %.2f, y: %.2f, th: %.2f", desired_offset.x, desired_offset.y, desired_offset.z);

    dynamic_reconfigure::Server<unicorn_docking::DockingControllerConfig> reconfig_server;

    reconfig_server.setCallback(boost::bind(&dynamicReconfigCallback, _1, _2));

    DockingController controller(n_pitch_avg);

    DockActionServer server(nh, "dock_action", boost::bind(&execute_action, _1, &server, &controller, nh), false);

    server.start();

    ros::spin();
    
    return 0;
}
