#include "ros/ros.h"
#include "chatter.pb.h"

void pbChatterCallback(const boost::shared_ptr<pb_msgs::ShortMessage>& msg)
{
  ROS_INFO_STREAM("Time: " << msg->stamp().sec() << "." << msg->stamp().nsec());
  ROS_INFO("I heard pb short message: [%s]", msg->content().c_str());
}

void counterCallback(const boost::shared_ptr<pb_msgs::Counter>& msg)
{
  ROS_INFO("I get counter message: [%d]", (int)msg->count());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber pb_sub = n.subscribe("pb_chatter", 1000, pbChatterCallback);
  ros::Subscriber sub = n.subscribe("counter", 1000, counterCallback);
  ros::spin();

  return 0;
}