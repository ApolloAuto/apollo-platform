# Design Doc: High Efficient Communication based on Shared Memory Transport

## Problem Definition

ROS supports both Node and Nodelet. For node, pub/sub services are based on socket because each node is a standalone process. For nodelet, pub/sub services are based on shared memory, nodelets are thread within the same process.

In autonomous driving area, we see a couple of problems in the original ROS design.

* For node, each pub/sub will have copies of message back and forth between application and kernel mode. When multiple subscriber for the same topic, the number of data copies are linear increased which leads to resource and performance concerns.
* For nodelet with thread mode, the isolation is worse than the node even though they have better communication based on shared memory.

## Feature Description

So we propose new Shared Memory Transport layer in ROS, which helps improve the latency and throughput between nodes. On top of this, we also support:
* Shared Memory based Transport and Socket based Transport co-exist, and publisher will choose the right transport based on the situation of the deployment.
* Shared Memory based Transport are enabled as default and will be preferred transport if conditions have been met.

## How to Use

For how to use this feature, checkout the [Example talker and listener](https://github.com/ApolloAuto/apollo-platform/tree/master/ros/ros_tutorials/roscpp_tutorials).

## FAQ

1. Is this transparent to the application?  
A: Yes, it is.

2. What if two nodes are on different machines?  
A: ROS will automatically decide which transport to use depending on the deployment. If two nodes are on different machines, socket based transport will be used.

3. How about nodelet?  
A: Nodelet are passing pointers already, it is compatible with this feature.

4. Python node?  
A: Python node is not supported in this feature yet. Python node will only use socket based transport.

5. How to turn off or turn on Shared Memory Transport?  
A: Please see the [config file](https://github.com/ApolloAuto/apollo-platform/blob/master/ros/ros_comm/roscpp/transport_mode.yaml). You can also find this config file in release package under "etc/ros/transport_mode.yaml".
