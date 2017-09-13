cmake_minimum_required(VERSION 2.8.3)
project(pcl_conversions)

find_package(catkin REQUIRED COMPONENTS pcl_msgs roscpp sensor_msgs std_msgs cmake_modules)

find_package(PCL REQUIRED QUIET COMPONENTS common)
find_package(Eigen REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_COMMON_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include ${PCL_COMMON_INCLUDE_DIRS}
  CATKIN_DEPENDS pcl_msgs roscpp sensor_msgs std_msgs
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Add gtest based cpp test target
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test test/test_pcl_conversions.cpp)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
endif()
