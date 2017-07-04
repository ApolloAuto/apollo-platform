cmake_minimum_required(VERSION 2.8.3)
project(message_generation)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS gencpp genlisp genmsg genpy)
