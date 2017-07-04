cmake_minimum_required(VERSION 2.8.3)
project(test_find_tinyxml)

find_package(catkin REQUIRED COMPONENTS cmake_modules)

find_package(TinyXML REQUIRED)

message("TinyXML_FOUND: ${TinyXML_FOUND}")
message("TinyXML_INCLUDE_DIRS: ${TinyXML_INCLUDE_DIRS}")
message("TinyXML_LIBRARIES: ${TinyXML_LIBRARIES}")

catkin_package(
  # INCLUDE ${TinyXML_INCLUDE_DIRS}  # Include this if one of your headers uses a header from TinyXML_INCLUDE_DIRS
  # LIBRARIES ${TinyXML_LIBRARIES}  # Include this if people who link against you need to link against TinyXML too
  # # OR
  # DEPENDS TinyXML  # Use this if you would use both of the above lines
)
