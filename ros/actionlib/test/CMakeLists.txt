if(GTEST_FOUND)
  include_directories(${GTEST_INCLUDE_DIRS})
  link_directories(${GTEST_LIBRARY_DIRS})

  add_executable(actionlib-add_two_ints_server EXCLUDE_FROM_ALL add_two_ints_server.cpp)
  target_link_libraries(actionlib-add_two_ints_server ${PROJECT_NAME} ${GTEST_LIBRARIES})

  add_executable(actionlib-ref_server EXCLUDE_FROM_ALL ref_server.cpp)
  target_link_libraries(actionlib-ref_server ${PROJECT_NAME})

  add_executable(actionlib-simple_client_test EXCLUDE_FROM_ALL simple_client_test.cpp)
  target_link_libraries(actionlib-simple_client_test ${PROJECT_NAME} ${GTEST_LIBRARIES})

  add_executable(actionlib-simple_execute_ref_server EXCLUDE_FROM_ALL simple_execute_ref_server.cpp)
  target_link_libraries(actionlib-simple_execute_ref_server ${PROJECT_NAME})

  add_executable(actionlib-server_goal_handle_destruction EXCLUDE_FROM_ALL server_goal_handle_destruction.cpp)
  target_link_libraries(actionlib-server_goal_handle_destruction ${PROJECT_NAME} ${GTEST_LIBRARIES})

  add_executable(actionlib-simple_client_wait_test EXCLUDE_FROM_ALL simple_client_wait_test.cpp)
  target_link_libraries(actionlib-simple_client_wait_test ${PROJECT_NAME} ${GTEST_LIBRARIES})

  add_executable(actionlib-simple_client_allocator_test EXCLUDE_FROM_ALL simple_client_allocator_test.cpp)
  target_link_libraries(actionlib-simple_client_allocator_test ${PROJECT_NAME} ${GTEST_LIBRARIES})

  add_executable(actionlib-action_client_destruction_test EXCLUDE_FROM_ALL action_client_destruction_test.cpp)
  target_link_libraries(actionlib-action_client_destruction_test ${PROJECT_NAME} ${GTEST_LIBRARIES})

  add_executable(actionlib-test_cpp_simple_client_cancel_crash EXCLUDE_FROM_ALL test_cpp_simple_client_cancel_crash.cpp)
  target_link_libraries(actionlib-test_cpp_simple_client_cancel_crash ${PROJECT_NAME} ${GTEST_LIBRARIES})

  add_executable(actionlib-exercise_simple_client EXCLUDE_FROM_ALL exercise_simple_client.cpp)
  target_link_libraries(actionlib-exercise_simple_client ${PROJECT_NAME} ${GTEST_LIBRARIES})

  if(TARGET tests)
    add_dependencies(tests
      actionlib-add_two_ints_server
      actionlib-ref_server
      actionlib-simple_client_test
      actionlib-simple_execute_ref_server
      actionlib-server_goal_handle_destruction
      actionlib-simple_client_wait_test
      actionlib-simple_client_allocator_test
      actionlib-action_client_destruction_test
      actionlib-test_cpp_simple_client_cancel_crash
      actionlib-exercise_simple_client
    )
  endif()
endif()

add_rostest(ref_server_test.launch)
add_rostest(simple_execute_ref_server_test.launch)
add_rostest(test_python_simple_client.launch)
add_rostest(test_cpp_simple_client_allocator.launch)
add_rostest(test_cpp_action_client_destruction.launch)
add_rostest(test_server_goal_handle_destruction.launch)
add_rostest(test_cpp_simple_client_cancel_crash.launch)
add_rostest(test_imports.launch)
add_rostest(test_python_server_components.launch)
add_rostest(test_python_server.launch)
add_rostest(test_python_server2.launch)
add_rostest(test_python_server3.launch)
add_rostest(test_python_simple_server.launch)
add_rostest(test_exercise_simple_clients.launch)
add_rostest(test_simple_action_server_deadlock_python.launch)

catkin_add_gtest(actionlib-destruction_guard_test destruction_guard_test.cpp)
if(TARGET actionlib-destruction_guard_test)
  target_link_libraries(actionlib-destruction_guard_test ${PROJECT_NAME})
endif()
