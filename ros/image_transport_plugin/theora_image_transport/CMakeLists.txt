cmake_minimum_required(VERSION 2.8.3)
project(theora_image_transport)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS cv_bridge dynamic_reconfigure image_transport message_generation rosbag pluginlib std_msgs)

add_message_files(DIRECTORY msg FILES Packet.msg)

generate_messages(DEPENDENCIES std_msgs)

find_package(PkgConfig)
pkg_check_modules(PC_OGG REQUIRED ogg)
pkg_check_modules(PC_THEORA REQUIRED theora)
pkg_check_modules(PC_THEORAENC REQUIRED theoraenc)
pkg_check_modules(PC_THEORADEC REQUIRED theoradec)


# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(cfg/TheoraPublisher.cfg cfg/TheoraSubscriber.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PC_OGG_INCLUDE_DIRS}
  ${PC_THEORA_INCLUDE_DIRS}
  ${PC_THEORAENC_INCLUDE_DIRS}
  ${PC_THEORADEC_INCLUDE_DIRS}
)

link_directories(${PC_OGG_LIBRARY_DIRS} 
                 ${PC_THEORA_LIBRARY_DIRS}
                 ${PC_THEORAENC_LIBRARY_DIRS}
                 ${PC_THEORADEC_LIBRARY_DIRS})
add_definitions(${PC_OGG_CFLAGS_OTHER} 
                ${PC_THEORA_CFLAGS_OTHER}
                ${PC_THEORAENC_CFLAGS_OTHER}
                ${PC_THEORADEC_CFLAGS_OTHER}
)

add_library(${PROJECT_NAME} src/theora_publisher.cpp src/theora_subscriber.cpp src/manifest.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
                                      ${PC_OGG_LIBRARIES}
                                      ${PC_THEORA_LIBRARIES}
                                      ${PC_THEORAENC_LIBRARIES}
                                      ${PC_THEORADEC_LIBRARIES}
)

class_loader_hide_library_symbols(${PROJECT_NAME})

add_executable(ogg_saver src/ogg_saver.cpp)
target_link_libraries(ogg_saver ${PC_THEORA_LIBRARY} 
                                ${PC_OGG_LIBRARY} 
                                ${OpenCV_LIBRARIES} 
                                ${catkin_LIBRARIES}  
                                ${PC_THEORAENC_LIBRARIES}
                                ${PC_THEORADEC_LIBRARIES})
add_dependencies(ogg_saver ${PROJECT_NAME}_gencpp)

install(TARGETS ${PROJECT_NAME} ogg_saver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# add xml file
install(FILES theora_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
