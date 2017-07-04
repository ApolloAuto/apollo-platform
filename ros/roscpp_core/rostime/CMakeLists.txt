cmake_minimum_required(VERSION 2.8.3)
project(rostime)

find_package(catkin REQUIRED COMPONENTS cpp_common)
find_package(Boost REQUIRED COMPONENTS date_time system thread)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cpp_common
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(rostime 
  src/duration.cpp
  src/rate.cpp
  src/time.cpp)

target_link_libraries(rostime ${Boost_LIBRARIES})
if(NOT APPLE)
  target_link_libraries(rostime ${RT_LIBRARY})
endif()

install(TARGETS rostime
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test_time test/time.cpp)
  if(TARGET ${PROJECT_NAME}-test_time)
    set_property(TARGET ${PROJECT_NAME}-test_time APPEND_STRING PROPERTY COMPILE_FLAGS "-std=c++11")
    target_link_libraries(${PROJECT_NAME}-test_time ${catkin_LIBRARIES} rostime)
  endif()
endif()
