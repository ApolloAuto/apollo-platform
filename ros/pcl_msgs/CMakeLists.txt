cmake_minimum_required(VERSION 2.8.3)
project(pcl_msgs)

# Deal with catkin
find_package(catkin REQUIRED COMPONENTS message_generation sensor_msgs std_msgs)

# create messages
add_message_files(DIRECTORY msg
                  FILES ModelCoefficients.msg
                        PointIndices.msg
                        PolygonMesh.msg
                        Vertices.msg
)
add_service_files(DIRECTORY srv
                  FILES UpdateFilename.srv)
generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime sensor_msgs std_msgs)
