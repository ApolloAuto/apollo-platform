^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package image_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.12 (2017-01-29)
--------------------
* Fix CMake of image_transport/tutorial and polled_camera
  Fix loads of problems with the CMakeLists.
* image_transport/tutorial: Add dependency on generated msg
  Without this, build fails on Kinetic because ResizedImage.h has not been
  generated yet.
* image_transport/tutorial: Add missing catkin_INCLUDE_DIRS
  Without this, compilation files on Kinetic because ros.h cannot be found.
* 1.11.11
* update changelogs
* Contributors: Martin Guenther, Vincent Rabaud

1.11.11 (2016-09-24)
--------------------

1.11.10 (2016-01-19)
--------------------

1.11.9 (2016-01-17)
-------------------
* fix linkage in tutorials
* Use $catkin_EXPORTED_TARGETS
* Contributors: Jochen Sprickerhof, Vincent Rabaud

1.11.8 (2015-11-29)
-------------------

1.11.7 (2015-07-28)
-------------------

1.11.6 (2015-07-16)
-------------------

1.11.5 (2015-05-14)
-------------------
* image_transport: fix CameraSubscriber shutdown (circular shared_ptr ref)
  CameraSubscriber uses a private boost::shared_ptr to share an impl object
  between copied instances. In CameraSubscriber::CameraSubscriber(), it
  handed this shared_ptr to boost::bind() and saved the created wall timer
  in the impl object, thus creating a circular reference. The impl object
  was therefore never freed.
  Fix that by passing a plain pointer to boost::bind().
* avoid a memory copy for the raw publisher
* add a way to publish an image with only the data pointer
* Make function inline to avoid duplicated names when linking statically
* add plugin examples for the tutorial
* update instructions for catkin
* remove uselessly linked library
  fixes `#28 <https://github.com/ros-perception/image_common/issues/28>`_
* add a tutorial for image_transport
* Contributors: Gary Servin, Max Schwarz, Vincent Rabaud

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
* update my email address
* Contributors: Vincent Rabaud

1.10.3 (2013-02-21 05:33)
-------------------------

1.10.2 (2013-02-21 04:48)
-------------------------

1.10.1 (2013-02-21 04:16)
-------------------------

1.10.0 (2013-01-13)
-------------------
* fix the urls
* use the pluginlib script to remove some warnings
* added license headers to various cpp and h files
* Contributors: Aaron Blasdel, Vincent Rabaud

1.9.22 (2012-12-16)
-------------------
* get rid of the deprecated class_loader interface
* Contributors: Vincent Rabaud

1.9.21 (2012-12-14)
-------------------
* CMakeLists.txt clean up
* Updated package.xml file(s) to handle new catkin buildtool_depend
  requirement
* Contributors: William Woodall, mirzashah

1.9.20 (2012-12-04)
-------------------

1.9.19 (2012-11-08)
-------------------
* add the right link libraries
* Contributors: Vincent Rabaud

1.9.18 (2012-11-06)
-------------------
* Isolated plugins into their own library to follow new
  class_loader/pluginlib guidelines.
* remove the brief attribute
* Contributors: Mirza Shah, Vincent Rabaud

1.9.17 (2012-10-30 19:32)
-------------------------

1.9.16 (2012-10-30 09:10)
-------------------------
* add xml file
* Contributors: Vincent Rabaud

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

1.9.3 (2012-09-12 20:44)
------------------------

1.9.2 (2012-09-10)
------------------

1.9.1 (2012-09-07 15:33)
------------------------
* make the libraries public
* Contributors: Vincent Rabaud

1.9.0 (2012-09-07 13:03)
------------------------
* catkinize for Groovy
* Initial image_common stack check-in, containing image_transport.
* Contributors: Vincent Rabaud, gerkey, kwc, mihelich, pmihelich, straszheim, vrabaud
