cmake_minimum_required(VERSION 2.8.3)
project(roscpp_serialization)
find_package(catkin REQUIRED COMPONENTS cpp_common roscpp_traits rostime)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES roscpp_serialization
  CATKIN_DEPENDS cpp_common roscpp_traits rostime
)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(roscpp_serialization src/serialization.cpp)

install(TARGETS roscpp_serialization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
