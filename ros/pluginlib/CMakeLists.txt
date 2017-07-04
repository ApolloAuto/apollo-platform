cmake_minimum_required(VERSION 2.8.3)
project(pluginlib)

find_package(catkin REQUIRED COMPONENTS class_loader rosconsole roslib cmake_modules)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(TinyXML REQUIRED)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS class_loader rosconsole roslib
  DEPENDS Boost TinyXML
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

add_executable(plugin_tool src/plugin_tool.cpp)
target_link_libraries(plugin_tool ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})

install(TARGETS plugin_tool
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/pluginlib/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES share/pluginlib/typed_class_loader_template.cpp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/plugin_macro_update
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  add_library(test_plugins EXCLUDE_FROM_ALL SHARED test/test_plugins.cpp)

  catkin_add_gtest(${PROJECT_NAME}_utest test/utest.cpp)
  if(TARGET ${PROJECT_NAME}_utest)
    target_link_libraries(${PROJECT_NAME}_utest ${TinyXML_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
    add_dependencies(${PROJECT_NAME}_utest test_plugins)
  endif()

  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  if(COMPILER_SUPPORTS_CXX11)
    catkin_add_gtest(${PROJECT_NAME}_unique_ptr_test test/unique_ptr_test.cpp)
    if(TARGET ${PROJECT_NAME}_unique_ptr_test)
      target_link_libraries(${PROJECT_NAME}_unique_ptr_test ${TinyXML_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
      set_target_properties(${PROJECT_NAME}_unique_ptr_test PROPERTIES COMPILE_FLAGS -std=c++11 LINK_FLAGS -std=c++11)
      add_dependencies(${PROJECT_NAME}_unique_ptr_test test_plugins)
    endif()
  endif()

endif()
