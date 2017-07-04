#include<sensor_msgs/SetCameraInfo.h>

int main()
{
  sensor_msgs::SetCameraInfo srv;
  srv.response.status_message = std::string("Some Text");

  std::cout << "Srv Response Msg: " << std::endl << srv.response << std::endl;

  uint8_t buf[1024];
  ros::serialization::OStream out(buf, sizeof(buf) );
  ros::serialization::serialize(out, srv.response);

  std::cout << "Its Serialized" << std::endl;

  sensor_msgs::SetCameraInfo::Response msg2;
  ros::serialization::IStream in(buf, sizeof(buf) );
  ros::serialization::deserialize(in, msg2);

  std::cout << "Im a message again: " << std::endl << msg2 << std::endl;


}
