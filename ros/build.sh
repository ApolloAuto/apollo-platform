#!/bin/bash
#
# Copyright (c) 2017, The Apollo Authors. All Rights Reserved
#
# VERSION:    1.0
# FILEIN:     ros
# FILEOUT:    install/ros_$MACHINE_ARCH

#------------------------ global variable --------------------------
CURRENT_PATH=`pwd`

MACHINE_ARCH="$(uname -m)"

INSTALL_PATH="${CURRENT_PATH}/install/ros_$MACHINE_ARCH"
FASTRTPS_PATH="${CURRENT_PATH}/third_party/fast-rtps"
TP_LIB_PATH="${CURRENT_PATH}/third_party/lib"

BUILD_TYPE="Release"
export LD_LIBRARY_PATH="${CURRENT_PATH}/third_party/fast-rtps/lib:$LD_LIBRARY_PATH"
#--------------------------- function ------------------------------
function info() {
  (>&2 echo -e "[\e[34m\e[1mINFO\e[0m] $*")
}

function error() {
  (>&2 echo -e "[\e[33m\e[1mERROR\e[0m] $*")
}

function ok() {
  (>&2 echo -e "[\e[32m\e[1m OK \e[0m] $*")
}

function print_delim() {
  echo '============================'
}

function success() {
  print_delim
  ok $1
  print_delim
}

function fail() {
  print_delim
  error $1
  print_delim
  exit -1
}

function print_usage() {
    echo 'Usage:
    ./build.sh [OPTION]'
    echo 'Options:
    build: run build only
    clean: run environment clean
    '
    return 0
}

function build() {
    build_ros
}

function clean() {
    clean_ros
}

function build_ros() {
    
    rm -f ${FASTRTPS_PATH} ${TP_LIB_PATH}
    ln -sf ${FASTRTPS_PATH}_${MACHINE_ARCH} ${FASTRTPS_PATH} &&
    ln -sf ${TP_LIB_PATH}_${MACHINE_ARCH} ${TP_LIB_PATH} &&

    ./catkin/bin/catkin_make_isolated --install --source . \
        --install-space ${INSTALL_PATH} -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
       -DFASTRTPS_PATH=${FASTRTPS_PATH} --cmake-args --no-warn-unused-cli &&
 
    cp -r ${CURRENT_PATH}/third_party/fast-rtps/lib/* ${INSTALL_PATH}/lib/ &&
    cp -r ${CURRENT_PATH}/third_party/lib/* ${INSTALL_PATH}/lib/ &&
    cp -r ${CURRENT_PATH}/third_party/bin/* ${INSTALL_PATH}/bin/ &&
    find -name "*.pyc" -print0 | xargs -0 rm -rf

    if [ $? -ne 0 ]; then
        fail 'Build ros failed!'
    fi
    success 'Build ros succeed!'
}

function clean_ros() {
    find -name "*.pyc" -print0 | xargs -0 rm -rf &&
    rm -rf .catkin_workspace &&
    rm -rf build_isolated &&
    rm -rf devel_isolated &&
    rm -rf install
    if [ $? -ne 0 ]; then
        fail 'Clean ros failed!'
    fi
    success 'Clean ros succeed!'
}

case $1 in
  build)
    build
    ;;
  clean)
    clean
    ;;
  *)
    print_usage
    ;;
esac

