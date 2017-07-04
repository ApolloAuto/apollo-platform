cmake_minimum_required(VERSION 2.8.3)
project(nodelet)

## Find catkin dependencies
find_package(catkin REQUIRED bondcpp cmake_modules message_generation pluginlib rosconsole roscpp std_msgs)

## Find Boost (only headers)
find_package(Boost REQUIRED)

## Find UUID libraries
find_package(UUID REQUIRED)

## Add service files to be generated
add_service_files(DIRECTORY srv FILES NodeletList.srv  NodeletLoad.srv  NodeletUnload.srv)

## Generate servics
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include ${UUID_INCLUDE_DIRS}
  LIBRARIES nodeletlib ${UUID_LIBRARIES}
  CATKIN_DEPENDS bondcpp message_runtime pluginlib rosconsole roscpp std_msgs
  DEPENDS Boost
)

include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})

# Debug only, collects stats on how callbacks are doled out to worker threads
#add_definitions(-DNODELET_QUEUE_DEBUG)

add_library(nodeletlib src/nodelet_class.cpp src/loader.cpp src/callback_queue.cpp src/callback_queue_manager.cpp)
target_link_libraries(nodeletlib ${catkin_LIBRARIES} ${BOOST_LIBRARIES})
add_dependencies(nodeletlib ${nodelet_EXPORTED_TARGETS})

add_executable(nodelet src/nodelet.cpp)
target_link_libraries(nodelet nodeletlib ${UUID_LIBRARIES} ${catkin_LIBRARIES} ${BOOST_LIBRARIES})

# install
catkin_install_python(PROGRAMS scripts/declared_nodelets scripts/list_nodelets
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS nodeletlib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS nodelet
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
