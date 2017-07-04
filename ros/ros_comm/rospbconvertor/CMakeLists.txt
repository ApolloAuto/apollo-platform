cmake_minimum_required(VERSION 2.8.3)
project(rospbconvertor)
find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/rospbconvertor
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
