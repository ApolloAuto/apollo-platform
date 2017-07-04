#swig -c++ -python participant.i

set -e
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR
MACHINE_ARCH=$(uname -m)
ROS_LIB="$DIR/../../install/ros_$MACHINE_ARCH/lib"
ROS_INCLUDE="$DIR/../../ros_comm/roscpp/include/discovery/"
g++ -shared -o _participant.so participant_wrap.cxx -std=c++11 -fpic -I/usr/include/python2.7/ -I../fast-rtps_$MACHINE_ARCH/include/ -I$ROS_INCLUDE -L$ROS_LIB -ldiscovery
echo "Success! copied _participant.so to $ROS_LIB"
cp _participant.so  $ROS_LIB
