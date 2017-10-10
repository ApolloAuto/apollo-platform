cmake_minimum_required(VERSION 2.8)
project(image_geometry)

find_package(catkin REQUIRED sensor_msgs)
find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS sensor_msgs
               DEPENDS OpenCV
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)

catkin_python_setup()

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# add a library
add_library(${PROJECT_NAME} src/pinhole_camera_model.cpp src/stereo_camera_model.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# add tests
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
