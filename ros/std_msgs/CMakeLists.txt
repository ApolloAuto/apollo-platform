cmake_minimum_required(VERSION 2.8.3)
project(std_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(
  DIRECTORY msg
  FILES
  Bool.msg
  Byte.msg
  ByteMultiArray.msg
  Char.msg
  ColorRGBA.msg
  Duration.msg
  Empty.msg
  Float32.msg
  Float32MultiArray.msg
  Float64.msg
  Float64MultiArray.msg
  Header.msg
  Int16.msg
  Int16MultiArray.msg
  Int32.msg
  Int32MultiArray.msg
  Int64.msg
  Int64MultiArray.msg
  Int8.msg
  Int8MultiArray.msg
  MultiArrayDimension.msg
  MultiArrayLayout.msg
  String.msg
  Time.msg
  UInt16.msg
  UInt16MultiArray.msg
  UInt32.msg
  UInt32MultiArray.msg
  UInt64.msg
  UInt64MultiArray.msg
  UInt8.msg
  UInt8MultiArray.msg
)

generate_messages()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime)

install(DIRECTORY include/std_msgs/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN "*.h")
