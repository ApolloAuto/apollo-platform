cmake_minimum_required(VERSION 2.8.3)
project(nodelet_topic_tools)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure)
find_package(Boost REQUIRED thread)

generate_dynamic_reconfigure_options(cfg/NodeletThrottle.cfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamic_reconfigure message_filters nodelet pluginlib roscpp
  DEPENDS Boost
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
