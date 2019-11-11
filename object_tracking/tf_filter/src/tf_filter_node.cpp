#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"

class PoseDrawer
{
public:
  PoseDrawer() : tf_(),  target_frame_("map")
  {
    point_sub_.subscribe(n_, "tx2/object_coordinates/point", 10);
	
	
    tf_filter_ = new tf::MessageFilter<geometry_msgs::PointStamped>(point_sub_, tf_, target_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&PoseDrawer::msgCallback, this, _1) );
  } ;
    ros::NodeHandle n_;
	geometry_msgs::PointStamped point_out;
private:
  message_filters::Subscriber<geometry_msgs::PointStamped> point_sub_;
  tf::TransformListener tf_;
  tf::MessageFilter<geometry_msgs::PointStamped> * tf_filter_;
  std::string target_frame_;

  //  Callback to register with tf::MessageFilter to be called when transforms are available
  void msgCallback(const boost::shared_ptr<const geometry_msgs::PointStamped>& point_ptr) 
  {    
    try  {
      tf_.transformPoint(target_frame_, *point_ptr, point_out);
    }
    catch (tf::TransformException &ex) {
      printf ("Failure %s\n", ex.what()); //Print exception which was caught
    }
  };

};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "tx2_tf_filter"); //Init ROS
  PoseDrawer pd; //Construct class
  ros::Publisher pubVector= pd.n_.advertise<geometry_msgs::PointStamped>("tx2/tf_filter/point",1);
	ros::Rate loop_rate(10);
	while (ros::ok()) {
		//publish message
		pubVector.publish(pd.point_out);
		ros::spinOnce();
		loop_rate.sleep();
	}
    return 0;
};
