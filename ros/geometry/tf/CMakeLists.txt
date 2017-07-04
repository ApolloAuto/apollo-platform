cmake_minimum_required(VERSION 2.8)
project(tf)

find_package(catkin REQUIRED)

find_package(Boost COMPONENTS thread signals system REQUIRED)
find_package(catkin COMPONENTS angles geometry_msgs message_filters message_generation rosconsole roscpp rostime sensor_msgs std_msgs tf2_ros REQUIRED)

catkin_python_setup()

include_directories(SYSTEM ${Boost_INCLUDE_DIR}
                           ${catkin_INCLUDE_DIRS}
)
include_directories(include)
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(DIRECTORY msg FILES tfMessage.msg)
add_service_files(DIRECTORY srv FILES FrameGraph.srv)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs message_filters message_runtime roscpp sensor_msgs std_msgs tf2_ros rosconsole
)

add_library(${PROJECT_NAME} src/tf.cpp src/transform_listener.cpp src/cache.cpp src/transform_broadcaster.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

# Debug
add_executable(tf_empty_listener src/empty_listener.cpp)
target_link_libraries(tf_empty_listener ${PROJECT_NAME})

add_executable(tf_echo src/tf_echo.cpp)
target_link_libraries(tf_echo ${PROJECT_NAME})

add_executable(tf_change_notifier src/change_notifier.cpp)
target_link_libraries(tf_change_notifier ${PROJECT_NAME})

add_executable(tf_monitor src/tf_monitor.cpp)
target_link_libraries(tf_monitor ${PROJECT_NAME})

add_executable(static_transform_publisher src/static_transform_publisher.cpp)
target_link_libraries(static_transform_publisher ${PROJECT_NAME})

# Tests
if(CATKIN_ENABLE_TESTING AND NOT ANDROID)

find_package(rostest)

catkin_add_gtest(tf_unittest test/tf_unittest.cpp)
target_link_libraries(tf_unittest ${PROJECT_NAME})

catkin_add_gtest(tf_quaternion_unittest test/quaternion.cpp)
target_link_libraries(tf_quaternion_unittest ${PROJECT_NAME})

catkin_add_gtest(test_transform_datatypes test/test_transform_datatypes.cpp)
target_link_libraries(test_transform_datatypes ${PROJECT_NAME} ${Boost_LIBRARIES})

add_executable(transform_listener_unittest test/transform_listener_unittest.cpp)
target_link_libraries(transform_listener_unittest ${PROJECT_NAME} ${GTEST_LIBRARIES})
add_rostest(test/transform_listener_unittest.launch)

# Disabled because of changes in TransformStorage
#catkin_add_gtest_future(tf_unittest_future test/tf_unittest_future.cpp)
#target_link_libraries(tf_unittest_future ${PROJECT_NAME})

catkin_add_gtest(test_velocity test/velocity_test.cpp)
target_link_libraries(test_velocity ${PROJECT_NAME})

#add_executable(test_transform_twist test/transform_twist_test.cpp)
#target_link_libraries(test_transform_twist ${PROJECT_NAME})
#catkin_add_gtest_build_flags(test_transform_twist)
#add_rostest(test/transform_twist_test.launch)

catkin_add_gtest(cache_unittest test/cache_unittest.cpp)
target_link_libraries(cache_unittest ${PROJECT_NAME})


add_executable(test_message_filter EXCLUDE_FROM_ALL test/test_message_filter.cpp)
target_link_libraries(test_message_filter tf ${Boost_LIBRARIES} ${GTEST_LIBRARIES})
add_rostest(test/test_message_filter.xml)


### Benchmarking
#catkin_add_gtest_future(tf_benchmark test/tf_benchmark.cpp)
#target_link_libraries(tf_benchmark ${PROJECT_NAME})

add_executable(testListener test/testListener.cpp)
target_link_libraries(testListener ${PROJECT_NAME} ${GTEST_LIBRARIES})
add_rostest(test/test_broadcaster.launch)

add_executable(testBroadcaster test/testBroadcaster.cpp)
target_link_libraries(testBroadcaster ${PROJECT_NAME})

endif()


if(NOT ANDROID)
#Python setup
set(Python_ADDITIONAL_VERSIONS 2.7)
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

# # If on Darwin, create a symlink _foo.so -> _foo.dylib, because the
# # MacPorts version of Python won't find _foo.dylib for 'import _foo'
# include(CMakeDetermineSystem)
# if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
#   add_custom_command(OUTPUT ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}_swig.so
#                      COMMAND cmake -E create_symlink  ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}_swig.dylib ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}_swig.so
#                      DEPENDS python_${PROJECT_NAME})
#   add_custom_target(symlink_darwin_lib ALL
#                     DEPENDS ${LIBRARY_OUTPUT_PATH}/_${PROJECT_NAME}_swig.so)
# endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")


# Check for SSE
#check_for_sse()

# Dynamic linking with tf worked OK, except for exception propagation, which failed in the unit test.
# so build with the objects directly instead.

link_libraries(${PYTHON_LIBRARIES})
add_library(pytf_py src/pytf.cpp src/tf.cpp src/transform_listener.cpp src/cache.cpp)
add_dependencies(pytf_py ${PROJECT_NAME}_gencpp)

find_package(PythonLibs REQUIRED)
set_target_properties(pytf_py PROPERTIES OUTPUT_NAME tf PREFIX "_" SUFFIX ".so"
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${PYTHON_INSTALL_DIR}/tf)


target_link_libraries(pytf_py ${Boost_LIBRARIES} ${catkin_LIBRARIES})
endif()

if(CATKIN_ENABLE_TESTING AND NOT ANDROID)
catkin_add_nosetests(test/testPython.py) 
add_executable(tf_speed_test EXCLUDE_FROM_ALL test/speed_test.cpp)
target_link_libraries(tf_speed_test ${PROJECT_NAME})
endif()


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME} tf_echo tf_empty_listener tf_change_notifier tf_monitor static_transform_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(NOT ANDROID)
install(TARGETS pytf_py
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}/tf)
endif()

# Install rosrun-able scripts
install(PROGRAMS 
  scripts/bullet_migration_sed.py
  scripts/tf_remap
  scripts/view_frames
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

if(TARGET tests)
  add_dependencies(tests testBroadcaster testListener transform_listener_unittest test_message_filter )
endif()
