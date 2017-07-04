cmake_minimum_required(VERSION 2.8.3)
project(pb_msgs)

find_package(catkin REQUIRED COMPONENTS genmsg gencpp genpy)

# catkin_python_setup()

add_proto_files(
    DIRECTORY proto
    FILES
    time.proto
)
generate_messages()

catkin_package(
#   INCLUDE_DIRS include
  LIBRARIES pb_msgs_proto
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS genmsg gencpp genpy
)

if(NOT EXISTS msg)
  file(MAKE_DIRECTORY msg)
endif()

install(DIRECTORY proto msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
