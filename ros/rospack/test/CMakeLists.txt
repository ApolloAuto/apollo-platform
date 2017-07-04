include_directories(${PROJECT_SOURCE_DIR}/src)
catkin_add_gtest(${PROJECT_NAME}-utest 
          test/utest.cpp ${PROJECT_SOURCE_DIR}/src/rospack_cmdline.cpp ${PROJECT_SOURCE_DIR}/src/utils.cpp
          WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
if(TARGET ${PROJECT_NAME}-utest)
  target_link_libraries(${PROJECT_NAME}-utest rospack)
endif()

# Prepare to run the tests.  This could be cleaner.
add_custom_target(${PROJECT_NAME}-prepare_test
                  COMMAND cmake -E copy_directory ${PROJECT_SOURCE_DIR}/test ${CMAKE_CURRENT_BINARY_DIR}
                  COMMAND cmake -E chdir test/deep "${PYTHON_EXECUTABLE}" deep.py)
if(TARGET ${PROJECT_NAME}-utest)
  add_dependencies(${PROJECT_NAME}-utest ${PROJECT_NAME}-prepare_test)
endif()

configure_file(test/utest.py.in
               ${CMAKE_CURRENT_BINARY_DIR}/test/utest.py)
configure_file(test/utest_rosstack.py.in
               ${CMAKE_CURRENT_BINARY_DIR}/test/utest_rosstack.py)
catkin_add_nosetests(${CMAKE_CURRENT_BINARY_DIR}/test
                      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test)
