cmake_minimum_required(VERSION 2.8)
project(polled_camera)

# generate the server
find_package(catkin REQUIRED COMPONENTS image_transport message_generation roscpp sensor_msgs std_msgs)

add_service_files(DIRECTORY srv FILES GetPolledImage.srv)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)

# define the project
catkin_package(CATKIN_DEPENDS image_transport message_runtime roscpp sensor_msgs std_msgs
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)


# create some library and exe
include_directories(include
                    ${catkin_INCLUDE_DIRS} 
                    ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

add_library(${PROJECT_NAME} src/publication_server.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(poller src/poller.cpp)
add_dependencies(poller ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(poller ${PROJECT_NAME}
                             ${catkin_LIBRARIES}
)

install(TARGETS poller
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
