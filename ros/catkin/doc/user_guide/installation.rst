Installation
============

If you have a working ROS installation, catkin should be installed
already.

Prerequisites
-------------

Catkin has the following dependencies:

* CMake - A cross-platform, open-source build system.
* Python - A general-purpose, interpreted high-level programming language.
* catkin_pkg - A Python runtime library for catkin.
* empy - A Python template library.
* nose - A Python testing framework.
* GTest - A C++ unit test framework from Google.

You can resolve these dependencies on Ubuntu with this command::

  sudo apt-get install cmake python-catkin-pkg python-empy python-nose libgtest-dev

If you are not on Ubuntu you can install ``catkin_pkg`` from PyPi via
pip::

  sudo pip install -U catkin_pkg

Ubuntu install
--------------

Catkin is distributed with ROS as binary debian package for Ubuntu, it
can be installed via::

  sudo apt-get install ros-groovy-catkin

For ROS Hydro, install ``ros-hydro-catkin`` instead.
