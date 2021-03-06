#include <ros/ros.h>
#include <gtest/gtest.h>

#include <unicorn_docking/docking_controller.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>

#include <geometry_msgs/Twist.h>

#include <std_msgs/Int32.h>

void setGains() {
    ros::NodeHandle n("~/pid");

    n.setParam("x/p", 6.0);
    n.setParam("x/i", 1.0);
    n.setParam("x/d", 2.0);
    n.setParam("x/i_max", 0.3);
    n.setParam("x/i_min", -0.3);

    n.setParam("th/p", 6.0);
    n.setParam("th/i", 1.0);
    n.setParam("th/d", 2.0);
    n.setParam("th/i_max", 0.3);
    n.setParam("th/i_min", -0.3);
}

apriltag_ros::AprilTagDetectionArray createForwardApriltagArray() {
    apriltag_ros::AprilTagDetection tag;
    apriltag_ros::AprilTagDetectionArray tagArray;
    tag.id.push_back(0);
    tag.size.push_back(0.035);
    tag.pose.pose.pose.position.x = -100.0;
    tag.pose.pose.pose.orientation.w = 1.0;

    tag.pose.header.frame_id = "realsense_depth_joint";
    tag.pose.header.stamp = ros::Time::now();

    tagArray.detections.push_back(tag);
    tagArray.header.frame_id = "realsense_depth_joint";
    tagArray.header.stamp = ros::Time::now();

    return tagArray;
}

TEST(DockingControllerTestSuite, ifNotDockingStateNoVelocity) {
    ros::NodeHandle n;
    setGains();

    DockingController controller;
    controller.setState(DockingController::DockState::IDLE);
    apriltag_ros::AprilTagDetectionArray tagArray = createForwardApriltagArray();


    ros::Publisher tagPub = n.advertise<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, false);
    tagPub.publish(tagArray);

    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    tagPub.shutdown();

    geometry_msgs::Twist vel;
    controller.computeVelocity(vel);

    //ASSERT_GT(controller.getDistanceToTag(), 1);
    ASSERT_FLOAT_EQ(vel.linear.x, 0.0);
}

TEST(DockingControllerTestSuite, ifDockingStateSomeVelocity) {
    ros::NodeHandle n;
    setGains();

    DockingController controller;
    controller.setState(DockingController::DockState::DOCKING);
    apriltag_ros::AprilTagDetectionArray tagArray = createForwardApriltagArray();

    ros::Publisher tagPub = n.advertise<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, false);
    tagPub.publish(tagArray);

    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    tagPub.shutdown();

    geometry_msgs::Twist vel;
    controller.computeVelocity(vel);

    //ASSERT_GT(controller.getDistanceToTag(), 1);
    ASSERT_LT(vel.linear.x, -0.05);
}

TEST(DockingControllerTestSuite, ifDockingStateAndNoTagNoVelocity) {
    ros::NodeHandle n;
    setGains();

    DockingController controller;

    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    controller.setState(DockingController::DockState::DOCKING);

    geometry_msgs::Twist vel;
    controller.computeVelocity(vel);

    ASSERT_FLOAT_EQ(vel.linear.x, 0.0);
}

TEST(DockingControllerTestSuite, canSetStateViaMessages) {
    ros::NodeHandle n("~");
    setGains();

    DockingController controller;

    // Make sure state is initially idle
    ASSERT_EQ(controller.getState(), DockingController::DockState::IDLE);

    ros::Publisher pub = n.advertise<std_msgs::Int32>("state", 1, false);
    std_msgs::Int32 msg;
    msg.data = DockingController::DockState::DOCKING;

    pub.publish(msg);

    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Make sure state is "docking"
    ASSERT_EQ(controller.getState(), DockingController::DockState::DOCKING);

    msg.data = DockingController::DockState::IDLE;

    pub.publish(msg);

    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    // Make sure state is "idle"
    ASSERT_EQ(controller.getState(), DockingController::DockState::IDLE);
}

TEST(DockingControllerTestSuite, setStateSetsState) {
    ros::NodeHandle n("~");
    setGains();

    DockingController controller;
    controller.setState(DockingController::DockState::IDLE);
    ASSERT_EQ(controller.getState(), DockingController::DockState::IDLE);

    controller.setState(DockingController::DockState::DOCKING);
    ASSERT_EQ(controller.getState(), DockingController::DockState::DOCKING);

    controller.setState(DockingController::DockState::IDLE);
    ASSERT_EQ(controller.getState(), DockingController::DockState::IDLE);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
