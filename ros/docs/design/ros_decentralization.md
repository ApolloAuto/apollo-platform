# Design Doc: Decentralization

## Problem Definition

ROS heavily relies on the Master role to manage all the communications, remote services and parameters across all the nodes. However, this leads to the concern on single node failure. If ROS master is down, then the whole system can not be recovered or resumed.

## Feature Description

To be free of dependency of the ROS master, we propose this decentralization feature, which implements:
* with this feature, only parameter services still stay in ROS master, all the other functions have been removed
* Even without ROS master, the node that does not depend on param, will still run without any problem
* tools like rostopic/rosnode still rely on ROS master to get stats information for better response time
* ROS master, as the parameter services, can persistent the data and recover the data after restart/crash. And the service discovery will help the new ROS master to recover information about topic/service/node/

## How to Use

For how to use this feature, you can checkout the [Example C++ Code](https://github.com/ApolloAuto/apollo-platform/tree/master/ros/ros_tutorials/roscpp_tutorials) or [Example Python Code](https://github.com/ApolloAuto/apollo-platform/tree/master/ros/ros_tutorials/rospy_tutorials).

## FAQ

1. What if the nodes can not communicate with each other?  
A: Service discovery use domain to partition each set of ROS master and its nodes. The domain is defined by the environment variable ROS_DOMAIN_ID.

2. Does it support deployment on multiple machines?  
A: As long as the two machines are setup with the same ROS_DOMAIN_ID and in the same network, they can communicate with each other.

3. No parameters are recovered after ROS master restarts?  
A: To enable recovery from the snapshot, you need to setup ROS_MASTER_SNAPSHOT, which is disabled as default.

4. Where is the snapshot file?  
A: $ROS_ROOT/.master_snapshot.
