cmake_minimum_required(VERSION 2.8.3)
project(actionlib)

find_package(catkin REQUIRED COMPONENTS actionlib_msgs message_generation roscpp rosunit std_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_python_setup()

add_action_files(DIRECTORY action FILES Test.action TestRequest.action TwoInts.action)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES actionlib
  CATKIN_DEPENDS actionlib_msgs message_runtime roscpp std_msgs
  DEPENDS Boost
)

add_library(actionlib src/connection_monitor.cpp src/goal_id_generator.cpp)
target_link_libraries(actionlib ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(actionlib actionlib_gencpp)

catkin_install_python(PROGRAMS tools/axclient.py tools/axserver.py tools/dynamic_action.py tools/library.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  add_subdirectory(test)
endif()
