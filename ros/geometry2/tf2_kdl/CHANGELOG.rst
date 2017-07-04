^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_kdl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.13 (2016-03-04)
-------------------
* converting python test script into unit test
* Don't export catkin includes
* Contributors: Jochen Sprickerhof, Tully Foote

0.5.12 (2015-08-05)
-------------------
* Add kdl::Frame to TransformStamped conversion
* Contributors: Paul Bovbel

0.5.11 (2015-04-22)
-------------------

0.5.10 (2015-04-21)
-------------------

0.5.9 (2015-03-25)
------------------

0.5.8 (2015-03-17)
------------------
* remove useless Makefile files
* fix ODR violations
* Contributors: Vincent Rabaud

0.5.7 (2014-12-23)
------------------
* fixing install rules and adding backwards compatible include with #warning
* Contributors: Tully Foote

0.5.6 (2014-09-18)
------------------

0.5.5 (2014-06-23)
------------------

0.5.4 (2014-05-07)
------------------

0.5.3 (2014-02-21)
------------------
* finding eigen from cmake_modules instead of from catkin
* Contributors: Tully Foote

0.5.2 (2014-02-20)
------------------
* add cmake_modules dependency for eigen find_package rules
* Contributors: Tully Foote

0.5.1 (2014-02-14)
------------------

0.5.0 (2014-02-14)
------------------

0.4.10 (2013-12-26)
-------------------

0.4.9 (2013-11-06)
------------------

0.4.8 (2013-11-06)
------------------

0.4.7 (2013-08-28)
------------------

0.4.6 (2013-08-28)
------------------

0.4.5 (2013-07-11)
------------------

0.4.4 (2013-07-09)
------------------
* making repo use CATKIN_ENABLE_TESTING correctly and switching rostest to be a test_depend with that change.

0.4.3 (2013-07-05)
------------------

0.4.2 (2013-07-05)
------------------

0.4.1 (2013-07-05)
------------------

0.4.0 (2013-06-27)
------------------
* moving convert methods back into tf2 because it does not have any ros dependencies beyond ros::Time which is already a dependency of tf2
* Cleaning up unnecessary dependency on roscpp
* converting contents of tf2_ros to be properly namespaced in the tf2_ros namespace
* Cleaning up packaging of tf2 including:
  removing unused nodehandle
  cleaning up a few dependencies and linking
  removing old backup of package.xml
  making diff minimally different from tf version of library
* Restoring test packages and bullet packages.
  reverting 3570e8c42f9b394ecbfd9db076b920b41300ad55 to get back more of the packages previously implemented
  reverting 04cf29d1b58c660fdc999ab83563a5d4b76ab331 to fix `#7 <https://github.com/ros/geometry_experimental/issues/7>`_
* passing unit tests

0.3.6 (2013-03-03)
------------------
* fix compilation under Oneiric

0.3.5 (2013-02-15 14:46)
------------------------
* 0.3.4 -> 0.3.5

0.3.4 (2013-02-15 13:14)
------------------------
* 0.3.3 -> 0.3.4

0.3.3 (2013-02-15 11:30)
------------------------
* 0.3.2 -> 0.3.3

0.3.2 (2013-02-15 00:42)
------------------------
* 0.3.1 -> 0.3.2
* fixed missing include export & tf2_ros dependecy

0.3.1 (2013-02-14)
------------------
* fixing version number in tf2_kdl
* catkinized tf2_kdl

0.3.0 (2013-02-13)
------------------
* fixing groovy-devel
* removing bullet and kdl related packages
* catkinizing geometry-experimental
* catkinizing tf2_kdl
* fix for kdl rotaiton constrition
* add twist, wrench and pose conversion to kdl, fix message to message conversion by adding specific conversion functions
* merge tf2_cpp and tf2_py into tf2_ros
* Got transform with types working in python
* A working first version of transforming and converting between different types
* Moving from camelCase to undescores to be in line with python style guides
* kdl unittest passing
* whitespace test
* add support for PointStamped geometry_msgs
* Fixing script
* set transform for test
* add advanced api
* working to transform kdl objects with dummy buffer_core
* plugin for py kdl
* add regression tests for geometry_msgs point, vector and pose
* add frame unit tests to kdl and bullet
* add first regression tests for kdl and bullet tf
* add bullet transforms, and create tests for bullet and kdl
* transform for vector3stamped message
* move implementation into library
* add advanced api
* compiling again with new design
* renaming classes
* compiling now
* almost compiling version of template code
* add test to start compiling
