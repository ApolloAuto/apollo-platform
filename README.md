# Apollo-Platform

The Apollo-platform will cover all the platform level support.
In the first release, we add the most popular solution Robot
Operating System (ROS) under ros directory.

## ROS

### Introduction

The Robot Operating System (ROS) is flexible framework for writing robot software.
This release is originated from ROS Indigo release.

### What is the difference

Compared to original ROS, we made the following improvements to enhance its stability and performance:

  * ROS Decentralization Feature
  * High Efficient Communication based on Shared Memory Transport Feature
  * Native Support with Protobuf Feature

For more details of each feature, please find in the [Design Docs](https://github.com/ApolloAuto/apollo-platform/blob/master/ros/docs/design).

The Apollo team would like to thank everybody in the open source community. The GitHub apollo-platform/ros is based on [ROS](https://github.com/ros/ros) and [Fast-RTPS](https://github.com/eProsima/Fast-RTPS). Currently, Apollo team maintains this repository. In near future, we’ll send patches back to the corresponding communities.

### License

All the features developed on ROS comply with the original ROS License which is BSD. However, those packages added for Apollo tools or examples are release Under Apache 2.0 License.

Under the current Apollo Platform release, there are two packages (orocos_kdl and xmlrpcpp) using LGPL license, which will only be dynamic linked into Apollo project. We are also releasing all the corresponding source code.

### How to Download the Release Package

Download the release packages from the release section on github:

```
https://github.com/ApolloAuto/apollo-platform/releases
```

### How to Install

After having the release package downloaded:

```
tar zxvf ros-indigo-apollo-2.0.0-x86_64.tar.gz
rsync -av ros/ /apollo/third_party/ros_x86_64
source /apollo/third_party/ros_x86_64/setup.bash
```

(Apollo docker is currently the only supported runtime environment)

### How to Build

If you would like customize and build your own ROS, please reference the [Apollo 2.0 Quick Start](https://github.com/ApolloAuto/apollo/blob/master/docs/quickstart/apollo_2_0_quick_start.md)
