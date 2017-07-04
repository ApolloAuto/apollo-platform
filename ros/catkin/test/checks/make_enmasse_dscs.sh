#!/usr/bin/env bash -ex

# this script deletes all of /opt/ros/fuerte and then runs cmake + gendebian

TOP=$(cd `dirname $0` ; pwd)
. $TOP/catkin-test.sh

CMAKE="cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/fuerte -DCMAKE_PREFIX_PATH=/opt/ros/fuerte -DCATKIN=YES -DCATKIN_DEB_SNAPSHOTS=YES -DCATKIN_PACKAGE_PREFIX=ros-fuerte-"
DESTDIR=$TOP/DESTDIR
CATKIN_DEB_SNAPSHOT_VERSION=$(date +%Y%m%d-%H%M%z)
export CATKIN_DEB_SNAPSHOT_VERSION

cleanup

cd $BUILD
$CMAKE '-DCATKIN_DPKG_BUILDPACKAGE_FLAGS=-d;-S;-kBE0A7693' $SRC
for distro in lucid maverick natty oneiric precise
do
    make VERBOSE=1 \
        CATKIN_DEB_SNAPSHOT_VERSION=$CATKIN_DEB_SNAPSHOT_VERSION \
        CATKIN_DEBIAN_DISTRIBUTION=$distro \
        gendebian
done

