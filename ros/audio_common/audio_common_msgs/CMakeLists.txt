cmake_minimum_required(VERSION 2.8.3)

project(audio_common_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)
add_message_files(DIRECTORY msg FILES AudioData.msg)
generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)
