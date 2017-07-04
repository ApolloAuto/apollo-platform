cmake_minimum_required(VERSION 2.8.3)
project(cpp_common)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(console_bridge REQUIRED)
find_package(catkin REQUIRED)
catkin_package(
  DEPENDS Boost console_bridge
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})

include(CheckIncludeFile)
include(CheckFunctionExists)
include(CheckCXXSourceCompiles)

include_directories(include ${Boost_INCLUDE_DIRS} ${console_bridge_INCLUDE_DIRS})

# execinfo.h is needed for backtrace on glibc systems
CHECK_INCLUDE_FILE(execinfo.h HAVE_EXECINFO_H)
if(HAVE_EXECINFO_H)
  add_definitions(-DHAVE_EXECINFO_H=1)
endif(HAVE_EXECINFO_H)
# do we have demangle capability?
# CHECK_INCLUDE_FILE doesn't work here for some reason
CHECK_CXX_SOURCE_COMPILES("#include<cxxabi.h>\nintmain(intargc,char**argv){}" HAVE_CXXABI_H)
if(HAVE_CXXABI_H)
  add_definitions(-DHAVE_CXXABI_H=1)
endif()
CHECK_FUNCTION_EXISTS(backtrace HAVE_GLIBC_BACKTRACE)
if(HAVE_GLIBC_BACKTRACE)
  add_definitions(-DHAVE_GLIBC_BACKTRACE)
endif(HAVE_GLIBC_BACKTRACE)

add_library(${PROJECT_NAME} src/debug.cpp src/header.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${console_bridge_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
