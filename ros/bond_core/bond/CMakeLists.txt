cmake_minimum_required(VERSION 2.8.3)
project(bond)

find_package(catkin REQUIRED message_generation std_msgs)

add_message_files(DIRECTORY msg
  FILES
  Constants.msg
  Status.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
