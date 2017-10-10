^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package camera_info_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.12 (2017-01-29)
--------------------
* 1.11.11
* update changelogs
* Return empty CameraInfo when !ros::ok()
* Contributors: Enrique Fernandez, Vincent Rabaud

1.11.11 (2016-09-24)
--------------------
* Return empty CameraInfo when !ros::ok()
* Contributors: Enrique Fernandez

1.11.10 (2016-01-19)
--------------------

1.11.9 (2016-01-17)
-------------------

1.11.8 (2015-11-29)
-------------------
* fix compilation on Fedora, fixes `#42 <https://github.com/ros-perception/image_common/issues/42>`_
* Contributors: Vincent Rabaud

1.11.7 (2015-07-28)
-------------------

1.11.6 (2015-07-16)
-------------------
* simplify target_link_libraries
  That should fix `#35 <https://github.com/ros-perception/image_common/issues/35>`_
* Contributors: Vincent Rabaud

1.11.5 (2015-05-14)
-------------------

1.11.4 (2014-09-21)
-------------------

1.11.3 (2014-05-19)
-------------------
* Add public member function to manually set camera info (`#19
  <https://github.com/ros-perception/image_common/issues/19>`_)
* make rostest in CMakeLists optional (`ros/rosdistro#3010
  <https://github.com/ros/rosdistro/issues/3010>`_)
* Contributors: Jack O'Quin, Jonathan Bohren, Lukas Bulwahn

1.11.2 (2014-02-13)
-------------------

1.11.1 (2014-01-26 02:33)
-------------------------
* check for CATKIN_ENABLE_TESTING
* Contributors: Lukas Bulwahn

1.11.0 (2013-07-20 12:23)
-------------------------

1.10.5 (2014-01-26 02:34)
-------------------------

1.10.4 (2013-07-20 11:42)
-------------------------
* add Jack as maintainer
* Contributors: Vincent Rabaud

1.10.3 (2013-02-21 05:33)
-------------------------
* add gtest libraries linkage
* Contributors: Vincent Rabaud

1.10.2 (2013-02-21 04:48)
-------------------------
* fix the rostest dependency
* Contributors: Vincent Rabaud

1.10.1 (2013-02-21 04:16)
-------------------------
* fix catkin gtest and rostest problem
* fix unit test dependencies
* Removed duplicated test dependancy
  Test dependencies should never duplicate build or run dependencies.
* Contributors: Aaron Blasdel, Jack O'Quin

1.10.0 (2013-01-13)
-------------------
* fix the urls
* Contributors: Vincent Rabaud

1.9.22 (2012-12-16)
-------------------

1.9.21 (2012-12-14)
-------------------
* Updated package.xml file(s) to handle new catkin buildtool_depend
  requirement
* Contributors: mirzashah

1.9.20 (2012-12-04)
-------------------

1.9.19 (2012-11-08)
-------------------

1.9.18 (2012-11-06)
-------------------
* remove the brief attribute
* Contributors: Vincent Rabaud

1.9.17 (2012-10-30 19:32)
-------------------------

1.9.16 (2012-10-30 09:10)
-------------------------

1.9.15 (2012-10-13 08:43)
-------------------------
* fix bad folder/libraries
* Contributors: Vincent Rabaud

1.9.14 (2012-10-13 01:07)
-------------------------

1.9.13 (2012-10-06)
-------------------

1.9.12 (2012-10-04)
-------------------

1.9.11 (2012-10-02 02:56)
-------------------------
* add missing rostest dependency
* Contributors: Vincent Rabaud

1.9.10 (2012-10-02 02:42)
-------------------------
* fix bad dependency
* Contributors: Vincent Rabaud

1.9.9 (2012-10-01)
------------------
* fix dependencies
* Contributors: Vincent Rabaud

1.9.8 (2012-09-30)
------------------
* add catkin as a dependency
* comply to the catkin API
* Contributors: Vincent Rabaud

1.9.7 (2012-09-18 11:39)
------------------------
* add missing linkage
* Contributors: Vincent Rabaud

1.9.6 (2012-09-18 11:07)
------------------------

1.9.5 (2012-09-13)
------------------
* install the include directories
* Contributors: Vincent Rabaud

1.9.4 (2012-09-12 23:37)
------------------------

1.9.3 (2012-09-12 20:44)
------------------------

1.9.2 (2012-09-10)
------------------
* fix build issues
* Contributors: Vincent Rabaud

1.9.1 (2012-09-07 15:33)
------------------------
* make the libraries public
* Contributors: Vincent Rabaud

1.9.0 (2012-09-07 13:03)
------------------------
* API documentation review update
* suppress misleading camera_info_manager error messages [`#5273
  <https://github.com/ros-perception/image_common/issues/5273>`_]
* remove deprecated global CameraInfoManager symbol for Fuerte (`#4971
  <https://github.com/ros-perception/image_common/issues/4971>`_)
* Revert to using boost::mutex, not boost::recursive_mutex.
* Hack saveCalibrationFile() to stat() the containing directory and
  attempt to create it if necessary.  Test for this case.
* Reload camera info when camera name changes.
* Implement most new Electric API changes, with test cases.
* Add ${ROS_HOME} expansion, with unit test cases.
  Do not use "$$" for a single '$', look for "${" instead.
* Use case-insensitive comparisons for parsing URL tags (`#4761
  <https://github.com/ros-perception/image_common/issues/4761>`_).
  Add unit test cases to cover this.  Add unit test case for camera
  name containing video mode.
* add test for resolving an empty URL
* Deprecate use of global CameraInfoManager symbol in E-turtle (`#4786
  <https://github.com/ros-perception/image_common/issues/4786>`_).
  Modify unit tests accordingly.
* provide camera_info_manager namespace, fixes `#4760
  <https://github.com/ros-perception/image_common/issues/4760>`_
* Add support for "package://" URLs.
* Fixed tests to work with new CameraInfo.
* Moved image_common from camera_drivers.
* Contributors: Vincent Rabaud, blaise, Jack O'Quin, mihelich
