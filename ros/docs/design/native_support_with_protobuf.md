# Design Doc: Native Support with Protobuf

## Problem Definition

ROS message is known having problem with backward compatibility. Anytime, when the ROS message is changed, the original collected data is not usable unless converted into the new message format. This is a huge pain in autonomous driving area because huge amount of data was recorded at runtime compared to the traditional robotic area. Protobuf is a popular protocol format, which is well designed with backward compatibility, also the data could be more compact. However, ROS community has concerns that protobuf will not handle very large message well. So, that is why we propose to support both original ROS message and protobuf natively.

## Feature Description

Adding native support with protobuf is not only about message itself, it also has broad range of impacts on all the existing tools, that is why this feature is more comprehensive than it looks like. In the future, we plan to support more tools from ROS distribution.

With this feature, we support:

* Compatibility
  * Just as original ROS message, you can publish/subscribe protobuf format message object or shared_ptr
  * rostopic echo can show the message content for both original ROS message or protobuf message
  * rosbag can record and reply for both original ROS message and protobuf message

* Easy to Use
  * Align with the catkin style, simply use add_proto_files to add protobuf message
  * support the build dependency across different packages
  * automatic deployment for cpp and python output

* Safety
  * At runtime, verify the message format for each topic 

## How to Use

For how to use the protobuf message, you can checkout the [Example Code](https://github.com/ApolloAuto/apollo-platform/tree/master/ros/pb_msgs_example).

## FAQ

1. why the datatype  of all the protobuf messages are like pb_msgs/xxxx?  
A: becuase protobuf has not idea about the relation between rosmsg and package, to be compatible with current ROS release, we deploy all the protobuf messages under pb_msgs package.

2. Do all the ROS tools support protobuf message?  
A: Some of the tools are tightly couple with original ROS message, by now the commands are not supported listed below:

| command|sub-command | support|
| ------------- |:-------------:| -----:|
| rostopic      | delay         | x |
| rostopic      | pub           | x |
| rosmsg        | show          | x |
