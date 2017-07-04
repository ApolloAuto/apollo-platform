cmake_minimum_required(VERSION 2.8.3)
project(rospack)

find_package(catkin REQUIRED COMPONENTS cmake_modules)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
set(Python_ADDITIONAL_VERSIONS "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")
find_package(PythonLibs "${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}" REQUIRED)
find_package(TinyXML REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rospack ${PYTHON_LIBRARIES}
  DEPENDS Boost TinyXML
)

#add_definitions(-Wall)

set(API_BACKCOMPAT_V1 "YES" CACHE BOOL "Whether to enable backwards compatibility with old C++ API")
if(API_BACKCOMPAT_V1)
  add_definitions(-DROSPACK_API_BACKCOMPAT_V1)
  set(backcompat_source src/rospack_backcompat.cpp)
endif()

include_directories(include ${TinyXML_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

add_library(rospack
  src/rospack.cpp
  ${backcompat_source}
  src/rospack_cmdline.cpp
  src/utils.cpp
)
target_link_libraries(rospack ${TinyXML_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

add_executable(rospackexe src/rospack_main.cpp)
# Set the name, and make it a "global" executable
set_target_properties(rospackexe PROPERTIES
  OUTPUT_NAME rospack
  RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/bin)
target_link_libraries(rospackexe rospack ${Boost_LIBRARIES})
add_executable(rosstackexe src/rosstack_main.cpp)
target_link_libraries(rosstackexe rospack ${Boost_LIBRARIES})
# Set the name, and make it a "global" executable
set_target_properties(rosstackexe PROPERTIES
  OUTPUT_NAME rosstack
  RUNTIME_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/bin)

install(TARGETS rospack rospackexe rosstackexe
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")

# uninstall target
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

if(DOXYGEN_FOUND)
  add_custom_target(rospack-docs
    COMMAND doxygen Doxyfile
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
endif()
