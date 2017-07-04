cmake_minimum_required(VERSION 2.8.3)
project(tf2_sensor_msgs)

find_package(catkin REQUIRED COMPONENTS cmake_modules sensor_msgs tf2_ros tf2)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Eigen)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS sensor_msgs tf2_ros tf2
   DEPENDS Eigen
)


include_directories(include
   ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

catkin_python_setup()

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_tf2_sensor_msgs.py)

find_package(catkin REQUIRED COMPONENTS sensor_msgs rostest tf2_ros tf2)

include_directories(${EIGEN_INCLUDE_DIRS})

add_executable(test_tf2_sensor_msgs_cpp EXCLUDE_FROM_ALL test/test_tf2_sensor_msgs.cpp)
target_link_libraries(test_tf2_sensor_msgs_cpp ${catkin_LIBRARIES} ${GTEST_LIBRARIES})


if(TARGET tests)
  add_dependencies(tests test_tf2_sensor_msgs_cpp)
endif()
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)


endif()
