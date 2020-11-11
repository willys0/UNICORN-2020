#include <ros/ros.h>
#include <gtest/gtest.h>

#include <unicorn_docking/docking_controller.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>

#include <geometry_msgs/Twist.h>


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
    DockingController controller;
    controller.setState(DockingController::DockState::IDLE);
    apriltag_ros::AprilTagDetectionArray tagArray = createForwardApriltagArray();


    ros::Publisher tagPub = n.advertise<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, false);
    tagPub.publish(tagArray);

    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    tagPub.shutdown();

    geometry_msgs::Twist vel = controller.computeVelocity();

    ASSERT_GT(controller.getDistanceToTag(), 1);
    ASSERT_FLOAT_EQ(vel.linear.x, 0.0);
}

TEST(DockingControllerTestSuite, ifDockingStateSomeVelocity) {
    ros::NodeHandle n;
    DockingController controller;
    controller.setState(DockingController::DockState::DOCKING);
    apriltag_ros::AprilTagDetectionArray tagArray = createForwardApriltagArray();

    ros::Publisher tagPub = n.advertise<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, false);
    tagPub.publish(tagArray);

    ros::WallDuration(0.5).sleep();
    ros::spinOnce();
    tagPub.shutdown();

    geometry_msgs::Twist vel = controller.computeVelocity();

    ASSERT_GT(controller.getDistanceToTag(), 1);
    ASSERT_LT(vel.linear.x, -0.05);
}

TEST(DockingControllerTestSuite, ifDockingStateAndNoTagNoVelocity) {
    ros::NodeHandle n;

    DockingController controller;

    ros::WallDuration(0.5).sleep();
    ros::spinOnce();

    controller.setState(DockingController::DockState::DOCKING);

    geometry_msgs::Twist vel = controller.computeVelocity();

    ASSERT_FLOAT_EQ(vel.linear.x, 0.0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
