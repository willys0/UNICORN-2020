#include <unicorn_roborio_bridge/uwb_interface.h>

UwbInterface::UwbInterface(ros::NodeHandle nh) :
    nh_(nh)
{
    initUwbMsg();

    uwb_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/uwb/pose", 10);
}

void UwbInterface::setUwbPosition(const geometry_msgs::Point& pos) {
    uwb_pose_msg_.header.stamp = ros::Time::now();
    uwb_pose_msg_.header.seq++;

    uwb_pose_msg_.pose.pose.position = pos;
    uwb_pose_msg_.pose.pose.position.x /= 1000.0;
    uwb_pose_msg_.pose.pose.position.y /= 1000.0;
    uwb_pose_msg_.pose.pose.position.z /= 1000.0;
}


void UwbInterface::publish() {
    // Should not be able to get zeros from UWB. If we get it anyway, something is not correct
    // so do not publish the message.
    if(uwb_pose_msg_.pose.pose.position.x != 0.0 ||
       uwb_pose_msg_.pose.pose.position.y != 0.0 ||
       uwb_pose_msg_.pose.pose.position.z != 0.0)
    {
        uwb_pub_.publish(uwb_pose_msg_);
    }

}

void UwbInterface::startTimer() {
    uwb_pub_timer_ = nh_.createTimer(ros::Duration(1.0 / uwb_pub_freq_), &UwbInterface::uwbPubTimeout, this);
}

void UwbInterface::initUwbMsg() {
    // TODO: Do we need to assign a frame to the header?
    nh_.param<std::string>("uwb/reference_frame", uwb_pose_msg_.header.frame_id, "");
    nh_.param<float>("uwb/publish_frequency", uwb_pub_freq_, 30.0);

    std::vector<double> covariance_vector;
    nh_.param<std::vector<double>>("uwb/covariance_matrix",
                                   covariance_vector,
                                   std::vector<double>({
                                       1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 0.0, 1.0
                                       }));

    // TODO: Incorporate orientation if we can get it later
    uwb_pose_msg_.pose.pose.orientation.x = 0;
    uwb_pose_msg_.pose.pose.orientation.y = 0;
    uwb_pose_msg_.pose.pose.orientation.z = 0;
    uwb_pose_msg_.pose.pose.orientation.w = 1;

    if(covariance_vector.size() != uwb_pose_msg_.pose.covariance.size()) {
        ROS_ERROR("[UWB settings] Covariance matrix not well formed, expected size: %lu, given size: %lu", uwb_pose_msg_.pose.covariance.size(), covariance_vector.size());
    }
    else {
        std::copy( covariance_vector.begin(), covariance_vector.end(), uwb_pose_msg_.pose.covariance.begin() );
    }

    ROS_INFO("[UWB settings] reference_frame: %s, publish_frequency: %f", uwb_pose_msg_.header.frame_id.c_str(), uwb_pub_freq_);
}

void UwbInterface::uwbPubTimeout(const ros::TimerEvent& e) {
    publish();
}
