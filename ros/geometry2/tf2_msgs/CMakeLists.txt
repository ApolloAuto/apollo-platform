cmake_minimum_required(VERSION 2.8.3)
project(tf2_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs actionlib_msgs)
find_package(Boost COMPONENTS thread REQUIRED)

add_message_files(DIRECTORY msg FILES TF2Error.msg TFMessage.msg)
add_service_files(DIRECTORY srv FILES FrameGraph.srv)

add_action_files(DIRECTORY action FILES LookupTransform.action)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_generation geometry_msgs actionlib_msgs)



