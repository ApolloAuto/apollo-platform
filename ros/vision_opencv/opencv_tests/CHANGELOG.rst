^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package opencv_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.15 (2017-01-29)
--------------------

1.11.14 (2016-09-24)
--------------------

1.11.13 (2016-07-11)
--------------------
* Support compressed Images messages in python for indigo
  - Add cv2_to_comprssed_imgmsg: Convert from cv2 image to compressed image ros msg.
  - Add comprssed_imgmsg_to_cv2:   Convert the compress message to a new image.
  - Add compressed image tests.
  - Add time to msgs (compressed and regular).
  add enumerants test for compressed image.
  merge the compressed tests with the regular ones.
  better comment explanation. I will squash this commit.
  Fix indentation
  fix typo mistage: from .imgmsg_to_compressed_cv2 to .compressed_imgmsg_to_cv2.
  remove cv2.CV_8UC1
  remove rospy and time depndency.
  change from IMREAD_COLOR to IMREAD_ANYCOLOR.
  - make indentaion of 4.
  - remove space trailer.
  - remove space from empty lines.
  - another set of for loops, it will make things easier to track. In that new set,  just have the number of channels in ([],1,3,4) (ignore two for jpg). from: https://github.com/ros-perception/vision_opencv/pull/132#discussion_r66721943
  - keep the OpenCV error message. from: https://github.com/ros-perception/vision_opencv/pull/132#discussion_r66721013
  add debug print for test.
  add case for 4 channels in test.
  remove 4 channels case from compressed test.
  add debug print for test.
  change typo of format.
  fix typo in format. change from dip to dib.
  change to IMREAD_ANYCOLOR as python code. (as it should).
  rename TIFF to tiff
  Sperate the tests one for regular images and one for compressed.
  update comment
* Contributors: talregev

1.11.12 (2016-03-10)
--------------------

1.11.11 (2016-01-31)
--------------------
* fix a few warnings in doc jobs
* Contributors: Vincent Rabaud

1.11.10 (2016-01-16)
--------------------

1.11.9 (2015-11-29)
-------------------

1.11.8 (2015-07-15)
-------------------
* simplify dependencies
* Contributors: Vincent Rabaud

1.11.7 (2014-12-14)
-------------------

1.11.6 (2014-11-16)
-------------------

1.11.5 (2014-09-21)
-------------------

1.11.4 (2014-07-27)
-------------------

1.11.3 (2014-06-08)
-------------------
* remove file whose functinality is now in cv_bridge
* remove references to cv (use cv2)
* Correct dependency from non-existent package to cv_bridge
* Contributors: Isaac Isao Saito, Vincent Rabaud

1.11.2 (2014-04-28)
-------------------

1.11.1 (2014-04-16)
-------------------

1.11.0 (2014-02-15)
-------------------

1.10.15 (2014-02-07)
--------------------

1.10.14 (2013-11-23 16:17)
--------------------------
* Contributors: Vincent Rabaud

1.10.13 (2013-11-23 09:19)
--------------------------
* Contributors: Vincent Rabaud

1.10.12 (2013-11-22)
--------------------
* Contributors: Vincent Rabaud

1.10.11 (2013-10-23)
--------------------
* Contributors: Vincent Rabaud

1.10.10 (2013-10-19)
--------------------
* Contributors: Vincent Rabaud

1.10.9 (2013-10-07)
-------------------
* Contributors: Vincent Rabaud

1.10.8 (2013-09-09)
-------------------
* update email  address
* Contributors: Vincent Rabaud

1.10.7 (2013-07-17)
-------------------

1.10.6 (2013-03-01)
-------------------

1.10.5 (2013-02-11)
-------------------

1.10.4 (2013-02-02)
-------------------

1.10.3 (2013-01-17)
-------------------

1.10.2 (2013-01-13)
-------------------

1.10.1 (2013-01-10)
-------------------
* fixes `#5 <https://github.com/ros-perception/vision_opencv/issues/5>`_ by removing the logic from Python and using wrapped C++ and adding a test for it
* Contributors: Vincent Rabaud

1.10.0 (2013-01-03)
-------------------

1.9.15 (2013-01-02)
-------------------

1.9.14 (2012-12-30)
-------------------

1.9.13 (2012-12-15)
-------------------

1.9.12 (2012-12-14)
-------------------
* Removed brief tag
  Conflicts:
  opencv_tests/package.xml
* buildtool_depend catkin fix
* Contributors: William Woodall

1.9.11 (2012-12-10)
-------------------

1.9.10 (2012-10-04)
-------------------

1.9.9 (2012-10-01)
------------------

1.9.8 (2012-09-30)
------------------

1.9.7 (2012-09-28 21:07)
------------------------
* add missing stuff
* make sure we find catkin
* Contributors: Vincent Rabaud

1.9.6 (2012-09-28 15:17)
------------------------
* move the test to where it belongs
* fix the tests and the API to not handle conversion from CV_TYPE to Color type (does not make sense)
* make all the tests pass
* comply to the new Catkin API
* backport the C++ test from Fuerte
* Contributors: Vincent Rabaud

1.9.5 (2012-09-15)
------------------
* remove dependencies to the opencv2 ROS package
* Contributors: Vincent Rabaud

1.9.4 (2012-09-13)
------------------

1.9.3 (2012-09-12)
------------------
* update to nosetests
* Contributors: Vincent Rabaud

1.9.2 (2012-09-07)
------------------
* be more compliant to the latest catkin
* added catkin_project() to cv_bridge, image_geometry, and opencv_tests
* Contributors: Jonathan Binney, Vincent Rabaud

1.9.1 (2012-08-28 22:06)
------------------------
* remove a deprecated header
* Contributors: Vincent Rabaud

1.9.0 (2012-08-28 14:29)
------------------------
* cleanup by Jon Binney
* catkinized opencv_tests by Jon Binney
* remove the version check, let's trust OpenCV :)
* revert the removal of opencv2
* finally get rid of opencv2 as it is a system dependency now
* bump REQUIRED version of OpenCV to 2.3.2, which is what's in ros-fuerte-opencv
* switch rosdep name to opencv2, to refer to ros-fuerte-opencv2
* Fixing link lines for gtest against opencv.
* Adding opencv2 to all manifests, so that client packages may
  not break when using them.
* baking in opencv debs and attempting a pre-release
* Another hack for prerelease to quiet test failures.
* Dissable a dubious opencv test. Temporary HACK.
* Changing to expect for more verbose failure.
* Minor change to test.
* Making this depend on libopencv-2.3-dev debian available in ros-shadow.
* mono16 -> bgr conversion tested and fixed in C
* Added Ubuntu platform tags to manifest
* Tuned for parc loop
* Demo of ROS node face detecton
* mono16 support, ticket `#2890 <https://github.com/ros-perception/vision_opencv/issues/2890>`_
* Remove use of deprecated rosbuild macros
* cv_bridge split from opencv2
* Name changes for opencv -> vision_opencv
* Validation for image message encoding
* utest changed to reflect rosimgtocv change to imgmsgtocv
* Add opencvpython as empty package
* New methods for cv image conversion
* Disabling tests on OSX, `#2769 <https://github.com/ros-perception/vision_opencv/issues/2769>`_
* New Python CvBridge, rewrote C CvBridge, regression test for C and Python CvBridge
* Fix underscore problem, test 8UC3->BGR8, fix 8UC3->BGR8
* New image format
* Image message and CvBridge change
* Rename rows,cols to height,width in Image message
* New node bbc for image testing
* Make executable
* Pong demo
* Missing utest.cpp
* New sensor_msgs::Image message
* Contributors: Vincent Rabaud, ethanrublee, gerkey, jamesb, jamesbowman, pantofaru, vrabaud, wheeler
