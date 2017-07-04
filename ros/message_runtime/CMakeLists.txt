cmake_minimum_required(VERSION 2.8.3)
project(message_runtime)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS cpp_common roscpp_serialization roscpp_traits rostime)
