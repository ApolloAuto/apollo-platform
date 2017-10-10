^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package polled_camera
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.12 (2017-01-29)
--------------------
* Fix CMake of image_transport/tutorial and polled_camera
  Fix loads of problems with the CMakeLists.
* 1.11.11
* update changelogs
* address gcc6 build error in polled_camera
  With gcc6, compiling fails with `stdlib.h: No such file or directory`,
  as including '-isystem /usr/include' breaks with gcc6, cf.,
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70129.
  This commit addresses this issue for this package in the same way
  it was addressed in various other ROS packages. A list of related
  commits and pull requests is at:
  https://github.com/ros/rosdistro/issues/12783
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@oss.bmw-carit.de>
* Contributors: Lukas Bulwahn, Martin Guenther, Vincent Rabaud

1.11.11 (2016-09-24)
--------------------
* address gcc6 build error in polled_camera
  With gcc6, compiling fails with `stdlib.h: No such file or directory`,
  as including '-isystem /usr/include' breaks with gcc6, cf.,
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70129.
  This commit addresses this issue for this package in the same way
  it was addressed in various other ROS packages. A list of related
  commits and pull requests is at:
  https://github.com/ros/rosdistro/issues/12783
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@oss.bmw-carit.de>
* Contributors: Lukas Bulwahn

1.11.10 (2016-01-19)
--------------------

1.11.9 (2016-01-17)
-------------------

1.11.8 (2015-11-29)
-------------------

1.11.7 (2015-07-28)
-------------------

1.11.6 (2015-07-16)
-------------------

1.11.5 (2015-05-14)
-------------------

1.11.4 (2014-09-21)
-------------------

1.11.3 (2014-05-19)
-------------------

1.11.2 (2014-02-13)
-------------------

1.11.1 (2014-01-26 02:33)
-------------------------

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

1.10.2 (2013-02-21 04:48)
-------------------------

1.10.1 (2013-02-21 04:16)
-------------------------

1.10.0 (2013-01-13)
-------------------
* use CATKIN_DEVEL_PREFIX instead of obsolete CATKIN_BUILD_PREFIX
* fix the urls
* Missing link flags exposed on OS X
* added license headers to various cpp and h files
* Contributors: Aaron Blasdel, Dirk Thomas, Vincent Rabaud, William Woodall

1.9.22 (2012-12-16)
-------------------
* replace genmsg by message_generation
* Contributors: Vincent Rabaud

1.9.21 (2012-12-14)
-------------------
* Updated package.xml file(s) to handle new catkin buildtool_depend requirement
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
* comlpy to the new catkin API
* Contributors: Vincent Rabaud

1.9.16 (2012-10-30 09:10)
-------------------------

1.9.15 (2012-10-13 08:43)
-------------------------
* fix bad folder/libraries
* Contributors: Vincent Rabaud

1.9.14 (2012-10-13 01:07)
-------------------------
* fix typo that resulted in bad installation of include folder
* Contributors: Vincent Rabaud

1.9.13 (2012-10-06)
-------------------

1.9.12 (2012-10-04)
-------------------

1.9.11 (2012-10-02 02:56)
-------------------------

1.9.10 (2012-10-02 02:42)
-------------------------

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

1.9.6 (2012-09-18 11:07)
------------------------

1.9.5 (2012-09-13)
------------------
* install the include directories
* Contributors: Vincent Rabaud

1.9.4 (2012-09-12 23:37)
------------------------
* make sure we depend on the server
* Contributors: Vincent Rabaud

1.9.3 (2012-09-12 20:44)
------------------------

1.9.2 (2012-09-10)
------------------

1.9.1 (2012-09-07 15:33)
------------------------

1.9.0 (2012-09-07 13:03)
------------------------
* catkinize for Groovy
* polled_camera (rep0104): Changed callback to allow filling
  status_message field.
* polled_camera (rep0104): Applied changes to GetPolledImage service.
* Contributors: Vincent Rabaud, eitan, gerkey, kwc, mihelich
