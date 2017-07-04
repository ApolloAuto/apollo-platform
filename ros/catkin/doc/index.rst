Catkin
======

`Catkin <http://wiki.ros.org/catkin>`_ is a collection of CMake macros
and associated code used to build packages used in `ROS
<http://www.ros.org>`_.

It was initially introduced as part of the ROS Fuerte_ release where
it was used for a small set of base packages.  For Groovy_ and Hydro_
it was significantly modified, and used by many more packages.  All
released Hydro packages were built using catkin, although existing
`rosbuild <http://wiki.ros.org/rosbuild>`_ packages can still be built
from source on top of the catkin packages.  Indigo_ is very similar,
except for some deprecated features that were removed.

.. note::

   This document covers the Indigo_ version.  The Groovy_ and Hydro_
   versions are `documented separately
   <http://docs.ros.org/hydro/api/catkin/html/>`_.

.. _Fuerte: http://wiki.ros.org/fuerte
.. _Groovy: http://wiki.ros.org/groovy
.. _Hydro: http://wiki.ros.org/hydro
.. _Indigo: http://wiki.ros.org/indigo


Contents
--------

.. toctree::
   :maxdepth: 2

   howto/index
   user_guide/user_guide
   adv_user_guide/adv_user_guide
   dev_guide/dev_guide

Code & support
--------------

+--------------+-------------------------------------+
| Catkin       | http://github.com/ros/catkin        |
+--------------+-------------------------------------+
| Issues       | http://github.com/ros/catkin/issues |
+--------------+-------------------------------------+
