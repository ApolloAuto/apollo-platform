cmake_minimum_required(VERSION 2.8.3)
project(cmake_modules)

find_package(catkin REQUIRED)

catkin_package(CFG_EXTRAS cmake_modules-extras.cmake)

## Install all cmake files
install(DIRECTORY cmake/Modules
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)
