#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tf2_ros'],
    package_dir={'': 'src'},
    requires=['rospy', 'actionlib', 'actionlib_msgs', 'tf2_msgs',
              'tf2_py', 'geometry_msgs']
)

setup(**d)
