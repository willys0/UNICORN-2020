#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>

#include <unicorn_docking/DockAction.h>

#include <unicorn_docking/docking_controller.h>

#include <actionlib/server/simple_action_server.h>

#include <dynamic_reconfigure/server.h>
#include <unicorn_docking/DockingControllerConfig.h>


typedef actionlib::SimpleActionServer<unicorn_docking::DockAction> DockActionServer;

enum DockStatus { RUNNING, SUCCESS, FAILED, ERROR };

double retry_offset;

ros::Publisher err_pub;
geometry_msgs::Point desired_offset;

int max_retries;
int max_error_times;
double thresh_x;
double thresh_y;
double thresh_th;

double max_docking_speed;
double max_undocking_speed;
double max_rotation_speed;


void dynamicReconfigCallback(unicorn_docking::DockingControllerConfig& config, uint32_t level, DockingController* controller) {
    thresh_x = config.x_error_thresh;
    thresh_y = config.y_error_thresh;
    thresh_th = config.th_error_thresh;

    max_docking_speed = config.max_docking_speed;
    max_undocking_speed = config.max_undocking_speed;
    max_rotation_speed = config.max_rotation_speed;

    controller->setDesiredRotationFunctionParameters(config.a, config.b, config.c);
    controller->setMaxTfLookupTime(config.max_tf_lookup_time);
    controller->setMinObjectDistances(config.min_dist_to_object_infront,config.min_dist_to_object_behind);
    controller->setRearLidarRotationMissalignment(config.rear_lidar_angle_offset);
    controller->setLidarContributionParameters(config.lidar_contrib_factor, config.lidar_contrib_offset);
}

DockStatus getDockingVelocity(ros::NodeHandle nh, DockingController* controller, DockActionServer* as, geometry_msgs::Point thresholds, geometry_msgs::Twist& out_velocity) {
    bool success;
    success = controller->computeVelocity(out_velocity);

    if(success == false) {
        return DockStatus::ERROR;
    }

    // cap the linear speed 
    if(out_velocity.linear.x > max_docking_speed) {
        out_velocity.linear.x = max_docking_speed;
    }
    else if(out_velocity.linear.x < -max_docking_speed) {
        out_velocity.linear.x = -max_docking_speed;
    }

    // cap the angular speed 
    if(out_velocity.angular.z > max_rotation_speed) {
        out_velocity.angular.z = max_rotation_speed;
    }
    else if(out_velocity.angular.z < -max_rotation_speed) {
        out_velocity.angular.z = -max_rotation_speed;
    }

    geometry_msgs::Vector3 pid_errors;

    pid_errors.x = fabs(controller->xError());
    pid_errors.y = fabs(controller->yError());
    pid_errors.z = fabs(controller->thError());
    err_pub.publish(pid_errors);

    if(pid_errors.x <= thresholds.x) {
        if(pid_errors.y > thresholds.y || pid_errors.z > thresholds.z) {
            return DockStatus::FAILED;
        }
        else {
            return DockStatus::SUCCESS;
        }
    }

    return DockStatus::RUNNING;

}

void getDockingResultMsg(DockingController* controller, bool success, unicorn_docking::DockResult& msg) {
    msg.forward_error = controller->xError();
    msg.lateral_error = controller->yError();
    msg.angular_error = controller->thError();
    msg.success = success;
}

void getDockingFeedbackMsg(DockingController* controller, unicorn_docking::DockFeedback& msg ) {
    msg.forward_error = controller->xError();
    msg.lateral_error = controller->yError();
    msg.angular_error = controller->thError();
}

void dock(ros::NodeHandle nh, DockingController* controller, DockActionServer* as) {
    ros::Publisher vel_pub;
    geometry_msgs::Twist move_msg;

    int remaining_retries = max_retries;

    DockStatus dock_status;

    unicorn_docking::DockResult rslt;
    unicorn_docking::DockFeedback fbk;

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);


    geometry_msgs::Point offset = desired_offset;
    geometry_msgs::Point thresh;
    thresh.x = thresh_x;
    thresh.y = thresh_y;
    thresh.z = thresh_th;

    bool docking = true;
    int error_times = 0;
    int sensor_error_times = 0;

    controller->reset();
    controller->setDesiredOffset(desired_offset);
    controller->setState(DockingController::DockState::DOCKING);

    ROS_INFO("[Docking Controller] Executing dock action...");

    while(true) {
        if(as->isPreemptRequested()) {
            getDockingResultMsg(controller, false, rslt);
            as->setPreempted(rslt);

            ROS_INFO("[Docking Controller] Dock action preempted with errors x: %f, y: %f, th: %f!", controller->xError(), controller->yError(), controller->thError());
            break;
        }

        // Call the docking controller to get velocity
        dock_status = getDockingVelocity(nh, controller, as, thresh, move_msg);

        if(dock_status == DockStatus::SUCCESS && docking == true) {
            getDockingResultMsg(controller, true, rslt);
            as->setSucceeded(rslt);

            ROS_INFO("[Docking Controller] Dock action finished with errors x: %f, y: %f, th: %f!", controller->xError(), controller->yError(), controller->thError());
            break;
        }
        else if(dock_status == DockStatus::FAILED && error_times++ > max_error_times) {

            // Keep trying to dock some number of times (undock and go back in)
            if(remaining_retries > 0) {
                if(docking) {
                    offset.x = retry_offset;
                    thresh.x = thresh_x * 4;

                    ROS_INFO("[Docking Controller] Failed to dock within acceptable error threshold, undocking to try again (remaining retries: %d)...", remaining_retries);
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
                // All retries failed, not good!
                getDockingResultMsg(controller, false, rslt);
                as->setPreempted(rslt);

                ROS_INFO("[Docking Controller] Dock action failed after %d retries with errors x: %f, y: %f, th: %f!", max_retries, controller->xError(), controller->yError(), controller->thError());

                break;
            }
        }
        else if(dock_status == DockStatus::ERROR && sensor_error_times++ > max_error_times){
                // Error during docking, not good!
                getDockingResultMsg(controller, false, rslt);
                as->setPreempted(rslt);

                ROS_INFO("[Docking Controller] Dock action failed due to outdated sensor data or transform error after %d retries with errors x: %f, y: %f, th: %f!", max_retries, controller->xError(), controller->yError(), controller->thError());

                break;
        }

        vel_pub.publish(move_msg);

        getDockingFeedbackMsg(controller, fbk);
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

void undock(ros::NodeHandle nh, DockingController* controller, DockActionServer* as) {
    ros::Publisher vel_pub;
    geometry_msgs::Twist move_msg;

    unicorn_docking::DockFeedback fbk;
    unicorn_docking::DockResult rslt;

    DockStatus dock_status;

    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    geometry_msgs::Point offset = desired_offset;
    offset.x = retry_offset;

    geometry_msgs::Point thresh;
    thresh.x = thresh_x * 4.0f;
    thresh.y = thresh_y;
    thresh.z = thresh_th;

    int sensor_error_times = 0;

    controller->reset();
    controller->setDesiredOffset(offset);
    controller->setState(DockingController::DockState::DOCKING);

    ROS_INFO("[Docking Controller] Executing undock action...");

    while(true) {
        if(as->isPreemptRequested()) {
            getDockingResultMsg(controller, false, rslt);
            as->setPreempted(rslt);

            ROS_INFO("[Docking Controller] Dock action preempted with errors x: %f, y: %f, th: %f!", controller->xError(), controller->yError(), controller->thError());
            break;
        }

        // Call the docking controller to get velocity
        dock_status = getDockingVelocity(nh, controller, as, thresh, move_msg);

        if(dock_status == DockStatus::SUCCESS) {
            getDockingResultMsg(controller, true, rslt);
            as->setSucceeded(rslt);

            ROS_INFO("[Docking Controller] Dock action finished with errors x: %f, y: %f, th: %f!", controller->xError(), controller->yError(), controller->thError());
            break;
        }
        else if(dock_status == DockStatus::FAILED) {
            getDockingResultMsg(controller, false, rslt);
            as->setPreempted(rslt);

            ROS_INFO("[Docking Controller] Undock action failed with errors x: %f, y: %f, th: %f!", controller->xError(), controller->yError(), controller->thError());

            break;
        }
        else if(dock_status == DockStatus::ERROR && sensor_error_times++ > max_error_times) {
                // Error during docking, not good!
                getDockingResultMsg(controller, false, rslt);
                as->setPreempted(rslt);

                ROS_INFO("[Docking Controller] Undock action failed due to outdated sensor data or transform error, error with errors x: %f, y: %f, th: %f!",controller->xError(), controller->yError(), controller->thError());

                break;
        }
        vel_pub.publish(move_msg);

        getDockingFeedbackMsg(controller, fbk);
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


void execute_action(const unicorn_docking::DockGoalConstPtr& goal, DockActionServer* as, DockingController* controller, ros::NodeHandle nh) {

    if(goal->undock) {
        undock(nh, controller, as);
    }
    else {
        dock(nh, controller, as);

    }
}

int main(int argc, char **argv) {

    // Init ros node
    ros::init(argc, argv, "unicorn_docking_node");

    ros::NodeHandle nh("~");

    // load settings
    nh.param("retry_error_times", max_error_times, 50);
    nh.param("max_retries", max_retries, 3);
    // nh.param("thresh_x", thresh_x, 0.01);
    // nh.param("thresh_y", thresh_y, 0.01);
    // nh.param("thresh_th", thresh_th, 0.01);

    nh.param("retry_offset", retry_offset, 0.6);
    nh.param("offset/x",desired_offset.x, 0.2);
    nh.param("offset/y",desired_offset.y, 0.0);
    nh.param("offset/th",desired_offset.z, 0.0);
    ROS_INFO("Setting dock offset to x: %.2f, y: %.2f, th: %.2f", desired_offset.x, desired_offset.y, desired_offset.z);

    err_pub = nh.advertise<geometry_msgs::Vector3>("errors", 1);

    dynamic_reconfigure::Server<unicorn_docking::DockingControllerConfig> reconfig_server;


    DockingController controller;

    reconfig_server.setCallback(boost::bind(&dynamicReconfigCallback, _1, _2, &controller));

    DockActionServer server(nh, "dock_action", boost::bind(&execute_action, _1, &server, &controller, nh), false);

    server.start();

    ros::spin();
    
    return 0;
}
