#!/usr/bin/env bash -ex

TOP=$(cd `dirname $0` ; pwd)
CATKIN_DEB_SNAPSHOT_VERSION=-$(date +%Y%m%d-%H%M%z)
export CATKIN_DEB_SNAPSHOT_VERSION

CMAKE="cmake -DCATKIN_PACKAGE_PREFIX=ros-fuerte- -DCATKIN_DEB_SNAPSHOTS=YES -DCMAKE_INSTALL_PREFIX=/opt/ros/fuerte -DCMAKE_PREFIX_PATH=/opt/ros/fuerte -DCATKIN=YES"
CHROOTDIR="/home/chroot/natty-amd64"
BUILDDIR=$TOP/debbuild
SRCDIR=$TOP/src
rm -rf $BUILDDIR || /bin/false
mkdir -p $BUILDDIR || /bin/false
REPODIR=$TOP/apt
SCHROOT="sudo schroot -d / -c natty-amd64"
SBUILD="sudo sbuild -A -m straszheim@willowgarage.com -d natty"
MAKE="make VERBOSE=1"
sudo mount --bind $REPODIR $CHROOTDIR/mnt/apt || /bin/true

scanrepo ()
{
    if [ `dirname $1` != $REPODIR ]  ; then
        cp $1 $REPODIR
    fi
    pushd $REPODIR
    [ -d conf ] || mkdir conf
    cat > conf/distributions <<EOF
Origin: Me
Label: lable
Codename: natty
Architectures: amd64
Components: main
Description: description of my repo
SignWith: yes
EOF
    reprepro includedeb natty $1 || /bin/false
    popd
    echo "deb file:///mnt/apt natty main" > /tmp/localrepo.list
    sudo cp /tmp/localrepo.list $CHROOTDIR/etc/apt/sources.list.d
    $SCHROOT apt-get update
}

cleanrepo () {
    rm -f /tmp/tmprepo/*.deb || /bin/true
    mkdir -p /tmp/tmprepo
    shopt -s nullglob
    for f in $REPODIR/*.deb ; do
        echo ">>> $f"
        cp $f /tmp/tmprepo
    done
    rm -rf $REPODIR/*
    sudo rm -f $CHROOTDIR/etc/apt/sources.list.d/localrepo.list
    $SCHROOT apt-get update

    gpg -a --export BE0A7693 > $CHROOTDIR/tmp/straszheim.key
    gpg -a --export BE0A7693 | $SCHROOT apt-key add -
    for f in /tmp/tmprepo/*.deb ; do
        cp $f $REPODIR
    done
    shopt -u nullglob
}

#cleanrepo

sudo dpkg -r ros-fuerte\* || /bin/true
sudo rm -rf /opt/ros/fuerte || /bin/true
sudo rm $REPODIR/*.deb || /bin/true

do_one () {
    dirname=$1
    debname=$2
    debversion=$3
    debplatform=$4

    fulldebname=ros-fuerte-${debname}_${debversion}~natty_${debplatform}.deb
    if [ -f $REPODIR/$fulldebname ] ; then
        sudo dpkg -i $REPODIR/$fulldebname
        scanrepo $REPODIR/$fulldebname
        return
    fi

    SRC=$SRCDIR/$dirname
    pushd $SRC
    [ -d .git ] && git clean -dfx
    popd

    mkdir $BUILDDIR/$dirname
    cd $BUILDDIR/$dirname
    rm -f CMakeCache.txt
    $CMAKE $SRC
    for d in lucid maverick natty oneiric
    do
        $MAKE CATKIN_DEBIAN_DISTRIBUTION=$d ${dirname}-gendebian
    done
    $MAKE ${dirname}-gendebian
    sudo sbuild -A -d natty $SRCDIR/ros-fuerte-${debname}_${debversion}${CATKIN_DEB_SNAPSHOT_VERSION}~natty.dsc
    sudo dpkg -i ros-fuerte-${debname}_${debversion}${CATKIN_DEB_SNAPSHOT_VERSION}~natty_${debplatform}.deb
    scanrepo ros-fuerte-${debname}_${debversion}${CATKIN_DEB_SNAPSHOT_VERSION}~natty_${debplatform}.deb
    dput ppa:straszheim/ros $SRCDIR/ros-fuerte-${debname}_${debversion}${CATKIN_DEB_SNAPSHOT_VERSION}~natty_source.changes
}

do_one catkin catkin 3.4.5 all
do_one genmsg genmsg 3.4.5 all
do_one gencpp gencpp 3.4.5 all
do_one genpy genpy 3.4.5 all
do_one std_msgs std-msgs 3.4.5 all
do_one common_msgs common-msgs 3.4.5 all
do_one roscpp_core roscpp-core 3.4.5 amd64
do_one ros_comm ros-comm 3.4.5 amd64




