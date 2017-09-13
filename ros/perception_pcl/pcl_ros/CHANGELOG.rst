^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pcl_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.7 (2015-06-08)
------------------
* Sync pcl_nodelets.xml from hydro to indigo
  Fixes to pass catkin lint -W1
* Fixes `#87 <https://github.com/ros-perception/perception_pcl/issues/87>`_ for Indigo
* Fixes `#85 <https://github.com/ros-perception/perception_pcl/issues/85>`_ for Indigo
* Fixes `#77 <https://github.com/ros-perception/perception_pcl/issues/77>`_ and `#80 <https://github.com/ros-perception/perception_pcl/issues/80>`_ for indigo
* Added option to save pointclouds in binary and binary compressed format
* Contributors: Lucid One, Mitchell Wills

1.2.6 (2015-02-04)
------------------

1.2.5 (2015-01-20)
------------------

1.2.4 (2015-01-15)
------------------

1.2.3 (2015-01-10)
------------------
* Update common.py
  Extended filter limits up to ±100000.0 in order to support intensity channel filtering.
* Contributors: Dani Carbonell

1.2.2 (2014-10-25)
------------------
* Adding target_frame
  [Ability to specify frame in bag_to_pcd ](https://github.com/ros-perception/perception_pcl/issues/55)
* Update pcl_nodelets.xml
  Included missing closing library tag.  This was causing the pcl/Filter nodelets below the missing nodelet tag to not be exported correctly.
* Contributors: Matt Derry, Paul Bovbel, Ruffin

1.2.1 (2014-09-13)
------------------
* clean up merge
* merge pull request `#60 <https://github.com/ros-perception/perception_pcl/issues/60>`_
* Contributors: Paul Bovbel

1.2.0 (2014-04-09)
------------------
* Updated maintainership
* Fix TF2 support for bag_to_pcd `#46 <https://github.com/ros-perception/perception_pcl/issues/46>`_
* Use cmake_modules to find eigen on indigo `#45 <https://github.com/ros-perception/perception_pcl/issues/45>`_

1.1.7 (2013-09-20)
------------------
* adding more uncaught config dependencies
* adding FeatureConfig dependency too

1.1.6 (2013-09-20)
------------------
* add excplicit dependency on gencfg target

1.1.5 (2013-08-27)
------------------
* Updated package.xml's to use new libpcl-all rosdep rules
* package.xml: tuned whitespaces
  This commit removes trailing whitespaces and makes the line with the license information in the package.xml bitwise match exactly the common license information line in most ROS packages.
  The trailing whitespaces were detected when providing a bitbake recipe in the meta-ros project (github.com/bmwcarit/meta-ros). In the recipe, the hash of the license line is declared and is used to check for changes in the license. For this recipe, it was not matching the common one.
  A related already merged commit is https://github.com/ros/std_msgs/pull/3 and a related pending commit is https://github.com/ros-perception/pcl_msgs/pull/1.

1.1.4 (2013-07-23)
------------------
* Fix a serialization error with point_cloud headers
* Initialize shared pointers before use in part of the pcl_conversions
  Should address runtime errors reported in `#29 <https://github.com/ros-perception/perception_pcl/issues/29>`_
* Changed the default bounds on filters to -1000, 1000 from -5, 5 in common.py

1.1.2 (2013-07-19)
------------------
* Fixed missing package exports on pcl_conversions and others
* Make find_package on Eigen and PCL REQUIRED

1.1.1 (2013-07-10)
------------------
* Add missing EIGEN define which caused failures on the farm

1.1.0 (2013-07-09)
------------------
* Add missing include in one of the installed headers
* Refactors to use pcl-1.7
* Use the PointIndices from pcl_msgs
* Experimental changes to point_cloud.h
* Fixes from converting from pcl-1.7, incomplete
* Depend on pcl_conversions and pcl_msgs
* bag_to_pcd: check return code of transformPointCloud()
  This fixes a bug where bag_to_pcd segfaults because of an ignored
  tf::ExtrapolationException.
* Changed #include type to lib
* Changed some #include types to lib
* removed a whitespace

1.0.34 (2013-05-21)
-------------------
* fixing catkin python imports

1.0.33 (2013-05-20)
-------------------
* Fixing catkin python imports

1.0.32 (2013-05-17)
-------------------
* Merge pull request `#11 <https://github.com/ros-perception/perception_pcl/issues/11>`_ from k-okada/groovy-devel
  revert removed directories
* fix to compileable
* copy features/segmentation/surface from fuerte-devel

1.0.31 (2013-04-22 11:58)
-------------------------
* No changes

1.0.30 (2013-04-22 11:47)
-------------------------
* deprecating bin install targets

1.0.29 (2013-03-04)
-------------------
* Fixes `#7 <https://github.com/ros-perception/perception_pcl/issues/7>`_
* now also works without specifying publishing interval like described in the wiki.

1.0.28 (2013-02-05 12:29)
-------------------------
* reenabling deprecated install targets - comment added

1.0.27 (2013-02-05 12:10)
-------------------------
* Update pcl_ros/package.xml
* Fixing target install directory for pcl tools
* update pluginlib macro

1.0.26 (2013-01-17)
-------------------
* fixing catkin export

1.0.25 (2013-01-01)
-------------------
* fixes `#1 <https://github.com/ros-perception/perception_pcl/issues/1>`_

1.0.24 (2012-12-21)
-------------------
* remove obsolete roslib import

1.0.23 (2012-12-19 16:52)
-------------------------
* clean up shared parameters

1.0.22 (2012-12-19 15:22)
-------------------------
* fix dyn reconf files

1.0.21 (2012-12-18 17:42)
-------------------------
* fixing catkin_package debs

1.0.20 (2012-12-18 14:21)
-------------------------
* adding catkin_project dependencies

1.0.19 (2012-12-17 21:47)
-------------------------
* adding nodelet_topic_tools dependency

1.0.18 (2012-12-17 21:17)
-------------------------
* adding pluginlib dependency
* adding nodelet dependencies
* CMake install fixes
* migrating nodelets and tools from fuerte release to pcl_ros
* Updated for new <buildtool_depend>catkin<...> catkin rule

1.0.17 (2012-10-26 09:28)
-------------------------
* remove useless tags

1.0.16 (2012-10-26 08:53)
-------------------------
* no need to depend on a meta-package

1.0.15 (2012-10-24)
-------------------
* do not generrate messages automatically

1.0.14 (2012-10-23)
-------------------
* bring back the PCL msgs

1.0.13 (2012-10-11 17:46)
-------------------------
* install library to the right place

1.0.12 (2012-10-11 17:25)
-------------------------

1.0.11 (2012-10-10)
-------------------
* fix a few dependencies

1.0.10 (2012-10-04)
-------------------
* comply to the new catkin API
* fixed pcl_ros manifest
* added pcl exports in manifest.xml
* fixed rosdeb pcl in pcl_ros/manifest.xml
* removing common_rosdeps from manifest.xml
* perception_pcl restructuring in groovy branch
* restructuring perception_pcl in groovy branch
* catkinized version of perception_pcl for groovy
* added PCL 1.6 stack for groovy
