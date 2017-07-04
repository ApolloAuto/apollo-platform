#include<sensor_msgs/PointCloud2.h>

int main()
{
  sensor_msgs::PointCloud2 pc_1;
  pc_1.width = 10;
  pc_1.height = 20;
  // todo set other stuff

  std::cout << "PointCloud2 message: " << std::endl << pc_1 << std::endl;

  uint8_t buf[1024];
  ros::serialization::OStream out(buf, sizeof(buf) );
  ros::serialization::serialize(out, pc_1);

  std::cout << "Message Was Serialized" << std::endl;

  sensor_msgs::PointCloud2 pc_2;
  ros::serialization::IStream in(buf, sizeof(buf) );
  ros::serialization::deserialize(in, pc_2);

  std::cout << "Its a message again: " << std::endl << pc_2 << std::endl;
}
