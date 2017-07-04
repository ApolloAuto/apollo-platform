cmake_minimum_required(VERSION 2.8)
project(eigen_conversions)

find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED cmake_modules geometry_msgs std_msgs)
find_package(Eigen REQUIRED)

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS include ${EIGEN_INCLUDE_DIRS}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs std_msgs
  DEPENDS orocos_kdl
)

add_library(${PROJECT_NAME}
  src/eigen_msg.cpp
  src/eigen_kdl.cpp
)
add_dependencies(${PROJECT_NAME} geometry_msgs_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
