cmake_minimum_required(VERSION 2.8.3)
project(tf2_kdl)

find_package(orocos_kdl)
find_package(catkin REQUIRED COMPONENTS cmake_modules tf2 tf2_ros tf2_msgs)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  DEPENDS Eigen orocos_kdl
)


catkin_python_setup()
link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(PROGRAMS scripts/test.py
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


if(CATKIN_ENABLE_TESTING)

find_package(catkin REQUIRED COMPONENTS rostest tf2 tf2_ros tf2_msgs)

add_executable(test_kdl EXCLUDE_FROM_ALL test/test_tf2_kdl.cpp)
find_package (Threads)
target_link_libraries(test_kdl ${catkin_LIBRARIES} ${GTEST_LIBRARIES} ${orocos_kdl_LIBRARIES} ${GTEST_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test.launch)
add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_python.launch)

if(TARGET tests)
  add_dependencies(tests test_kdl)
endif()

endif()
