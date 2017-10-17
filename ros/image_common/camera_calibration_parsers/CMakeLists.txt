cmake_minimum_required(VERSION 2.8)
project(camera_calibration_parsers)

find_package(catkin REQUIRED sensor_msgs rosconsole roscpp roscpp_serialization)

find_package(PythonLibs REQUIRED)
if(PYTHONLIBS_VERSION_STRING VERSION_LESS 3)
  find_package(Boost REQUIRED COMPONENTS filesystem python)
else()
  find_package(Boost REQUIRED COMPONENTS filesystem python3)
endif()
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS sensor_msgs
)

find_package(PkgConfig)


if (ANDROID)
    find_package(yaml-cpp)
    add_definitions(-DHAVE_NEW_YAMLCPP)
else()
  pkg_check_modules(YAML_CPP yaml-cpp)
  if(${YAML_CPP_VERSION} VERSION_GREATER 0.5)
    add_definitions(-DHAVE_NEW_YAMLCPP)
  endif()
  link_directories(${YAML_CPP_LIBRARY_DIRS})
endif()
include_directories(${YAML_CPP_INCLUDE_DIRS})

# define the library
add_library(${PROJECT_NAME}
  src/parse.cpp
  src/parse_ini.cpp
  src/parse_yml.cpp
)

add_library(${PROJECT_NAME}_wrapper
  src/parse_wrapper.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY})
target_link_libraries(${PROJECT_NAME}_wrapper ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(${PROJECT_NAME}_wrapper PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
        )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  COMPONENT main
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# define the exe to convert
add_executable(convert src/convert.cpp)
target_link_libraries(convert ${PROJECT_NAME} ${rosconsole_LIBRARIES})

install(
  TARGETS convert
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_wrapper
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
