^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package image_geometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.15 (2017-01-29)
--------------------
* Import using __future_\_ for python 3 compatibility.
* Contributors: Hans Gaiser

1.11.14 (2016-09-24)
--------------------
* Fix "stdlib.h: No such file or directory" errors in GCC-6
  Including '-isystem /usr/include' breaks building with GCC-6.
  See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70129
* remap with nan border if mat value is float or double
* remap with nan border if mat value is float or double
* Contributors: Hodorgasm, YuOhara

1.11.13 (2016-07-11)
--------------------
* Add fullResolution getter to PinholeCameraModel
* add a missing dependency when building the doc
* fix sphinx doc path
* Contributors: Jacob Panikulam, Vincent Rabaud

1.11.12 (2016-03-10)
--------------------
* issue `#117 <https://github.com/ros-perception/vision_opencv/issues/117>`_ pull request `#118 <https://github.com/ros-perception/vision_opencv/issues/118>`_ check all distortion coefficients to see if rectification ought to be done
* Contributors: Lucas Walter

1.11.11 (2016-01-31)
--------------------
* clean up the doc files
* fix a few warnings in doc jobs
* Contributors: Vincent Rabaud

1.11.10 (2016-01-16)
--------------------

1.11.9 (2015-11-29)
-------------------
* add a condition if D=None
* fix compilation warnings
* Contributors: Vincent Rabaud, YuOhara

1.11.8 (2015-07-15)
-------------------
* fixes `#62 <https://github.com/ros-perception/vision_opencv/issues/62>`_, bug in Python rectifyPoint old opencv1 API
* Simplify some OpenCV3 distinction
* Contributors: Basheer Subei, Vincent Rabaud

1.11.7 (2014-12-14)
-------------------
* Merge pull request `#53 <https://github.com/ros-perception/vision_opencv/issues/53>`_ from carnegieroboticsllc/patch-1
  Update stereo_camera_model.cpp
* Updated inline math for reprojecting a single disparity
* Update stereo_camera_model.cpp
  Correct slight error in the Q matrix derivation
* Updated Q matrix to account for cameras with different Fx and Fy values
* Contributors: Carnegie Robotics LLC, Matt Alvarado, Vincent Rabaud

1.11.6 (2014-11-16)
-------------------
* Fixes in image_geometry for Python cv2 API
* Fixed typo: np -> numpy
* Added missing tfFrame method to Python PinholeCameraModel.
* Removed trailing whitespace.
* Contributors: Daniel Maturana

1.11.5 (2014-09-21)
-------------------
* get code to work with OpenCV3
  actually fixes `#46 <https://github.com/ros-perception/vision_opencv/issues/46>`_ properly
* Contributors: Vincent Rabaud

1.11.4 (2014-07-27)
-------------------

1.11.3 (2014-06-08)
-------------------
* pinhole_camera_model: fix implicit shared_ptr cast to bool for C++11
  In C++11 boost::shared_ptr does not provide the implicit bool conversion
  operator anymore, so make the cast in pinhole_camera_model.h explicit.
  That does not hurt in older C++ standards and makes compilation with C++11
  possible.
* Contributors: Max Schwarz

1.11.2 (2014-04-28)
-------------------

1.11.1 (2014-04-16)
-------------------

1.11.0 (2014-02-15)
-------------------
* remove OpenCV 1 API
* fixes `#6 <https://github.com/ros-perception/vision_opencv/issues/6>`_
* fix OpenCV dependencies
* Contributors: Vincent Rabaud

1.10.15 (2014-02-07)
--------------------
* add assignment operator for StereoCameraModel
* fixed Python rectifyImage implementation in PinholeCameraModel
* Added operator= for the PinholeCameraModel.
  Added the operator= for the PinholeCameraModel. I am not sure if the
  PinholeCameraModel needs to have a destructor, too. To follow the
  'rule of three' it should actually have one.
* Contributors: Tobias Bär, Valsamis Ntouskos, Vincent Rabaud

1.10.14 (2013-11-23 16:17)
--------------------------
* Contributors: Vincent Rabaud

1.10.13 (2013-11-23 09:19)
--------------------------
* Contributors: Vincent Rabaud

1.10.12 (2013-11-22)
--------------------
* "1.10.12"
* Contributors: Vincent Rabaud

1.10.11 (2013-10-23)
--------------------
* Contributors: Vincent Rabaud

1.10.10 (2013-10-19)
--------------------
* Contributors: Vincent Rabaud

1.10.9 (2013-10-07)
-------------------
* fixes `#23 <https://github.com/ros-perception/vision_opencv/issues/23>`_
* Contributors: Vincent Rabaud

1.10.8 (2013-09-09)
-------------------
* check for CATKIN_ENABLE_TESTING
* update email  address
* Contributors: Lukas Bulwahn, Vincent Rabaud

1.10.7 (2013-07-17)
-------------------

1.10.6 (2013-03-01)
-------------------

1.10.5 (2013-02-11)
-------------------
* Add dependency on generated messages
  Catkin requires explicit enumeration of dependencies on generated messages.
  Add this declaration to properly flatten the dependency graph and force Catkin
  to generate geometry_msgs before compiling image_geometry.
* Contributors: Adam Hachey

1.10.4 (2013-02-02)
-------------------

1.10.3 (2013-01-17)
-------------------

1.10.2 (2013-01-13)
-------------------
* fix ticket 4253
* Contributors: Vincent Rabaud

1.10.1 (2013-01-10)
-------------------

1.10.0 (2013-01-03)
-------------------

1.9.15 (2013-01-02)
-------------------

1.9.14 (2012-12-30)
-------------------
* add feature for https://code.ros.org/trac/ros-pkg/ticket/5592
* CMake cleanups
* fix a failing test
* Contributors: Vincent Rabaud

1.9.13 (2012-12-15)
-------------------
* use the catkin macros for the setup.py
* Contributors: Vincent Rabaud

1.9.12 (2012-12-14)
-------------------
* buildtool_depend catkin fix
* Contributors: William Woodall

1.9.11 (2012-12-10)
-------------------
* Fixing image_geometry package.xml
* fix https://code.ros.org/trac/ros-pkg/ticket/5570
* Contributors: Vincent Rabaud, William Woodall

1.9.10 (2012-10-04)
-------------------

1.9.9 (2012-10-01)
------------------
* fix dependencies
* Contributors: Vincent Rabaud

1.9.8 (2012-09-30)
------------------
* fix some dependencies
* fix missing Python at install and fix some dependencies
* Contributors: Vincent Rabaud

1.9.7 (2012-09-28 21:07)
------------------------
* add missing stuff
* make sure we find catkin
* Contributors: Vincent Rabaud

1.9.6 (2012-09-28 15:17)
------------------------
* make all the tests pass
* comply to the new Catkin API
* Contributors: Vincent Rabaud

1.9.5 (2012-09-15)
------------------
* remove dependencies to the opencv2 ROS package
* Contributors: Vincent Rabaud

1.9.4 (2012-09-13)
------------------
* make sure the include folders are copied to the right place
* Contributors: Vincent Rabaud

1.9.3 (2012-09-12)
------------------

1.9.2 (2012-09-07)
------------------
* be more compliant to the latest catkin
* added catkin_project() to cv_bridge, image_geometry, and opencv_tests
* Contributors: Jonathan Binney, Vincent Rabaud

1.9.1 (2012-08-28 22:06)
------------------------
* remove things that were marked as ROS_DEPRECATED
* Contributors: Vincent Rabaud

1.9.0 (2012-08-28 14:29)
------------------------
* catkinized opencv_tests by Jon Binney
* fix ticket 5449
* use OpenCV's projectPoints
* remove the version check, let's trust OpenCV :)
* revert the removal of opencv2
* vision_opencv: Export OpenCV flags in manifests for image_geometry, cv_bridge.
* finally get rid of opencv2 as it is a system dependency now
* bump REQUIRED version of OpenCV to 2.3.2, which is what's in ros-fuerte-opencv
* switch rosdep name to opencv2, to refer to ros-fuerte-opencv2
* Adding a few missing headers so that client code may compile against pinhole camera model.
* Adding opencv2 to all manifests, so that client packages may
  not break when using them.
* baking in opencv debs and attempting a pre-release
* image_geometry: (Python) Adjust K and P for ROI/binning. Also expose full resolution K and P. Add raw_roi property.
* image_geometry: Add Tx, Ty getters (Python).
* image_geometry: Added tf_frame and stamp properties. Only generate undistort maps when rectifyImage is called.
* image_geometry: Fix for when D is empty (Python).
* image_geometry: Take all D coefficients, not just the first 4 (Python).
* image_geometry: Fix rectification in the presence of binning (`#4848 <https://github.com/ros-perception/vision_opencv/issues/4848>`_).
* image_geometry: Fixed wg-ros-pkg `#5019 <https://github.com/ros-perception/vision_opencv/issues/5019>`_, error updating StereoCameraModel. Removed manifest dependency on cv_bridge.
* image_geometry: fromCameraInfo() returns bool, true if parameters have changed since last call.
* image_geometry: Accessors for full-res K, P.
* image_geometry: Implemented toFullResolution(), toReducedResolution().
* image_geometry: Implemented reducedResolution().
* image_geometry: Implemented rectifiedRoi() with caching. Fixed bug that would cause rectification maps to be regenerated every time.
* image_geometry: Implemented rectifyRoi().
* image_geometry: Overloads of projection functions that return the output directly instead of through a reference parameter. Implemented unrectifyRoi(). Added fullResolution(), rawRoi().
* image_geometry: Library-specific exception class.
* image_geometry: PIMPL pattern for cached data, so I can change in patch releases if necessary. Changed projectPixelTo3dRay() to normalize to z=1.
* image_geometry (rep0104): Added binning. Partially fixed ROI (not finding rectified ROI yet). Now interpreting distortion_model. Lots of code cleanup.
* image_geometry (rep0104): Got tests passing again, were issues with D becoming variable-length.
* image_geometry: Fixed swapped width/height in computing ROI undistort maps. Partially fixes `#4206 <https://github.com/ros-perception/vision_opencv/issues/4206>`_.
* image_geometry: getDelta functions, getZ and getDisparity in C++ and Python. Docs and tests for them. Changed Python fx() and friends to pull values out of P instead of K.
* image_geometry: Added C++ getDeltaU and getDeltaV.
* `#4201 <https://github.com/ros-perception/vision_opencv/issues/4201>`_, implement/doc/test for getDeltaU getDeltaX getDeltaV getDeltaY
* Added Ubuntu platform tags to manifest
* `#4083 <https://github.com/ros-perception/vision_opencv/issues/4083>`_, projectPixelTo3dRay implemented
* image_geometry: Added PinholeCameraModel::stamp() returning the time stamp.
* image_geometry: Fixed bugs related to ignoring Tx & Ty in projectPixelTo3dRay and unrectifyPoint. Added Tx() and Ty() accessors.
* image_geometry: Fixed `#4063 <https://github.com/ros-perception/vision_opencv/issues/4063>`_, PinholeCameraModel ignores Tx term in P matrix.
* image_geometry: Implemented projectDisparityTo3d, `#4019 <https://github.com/ros-perception/vision_opencv/issues/4019>`_.
* image_geometry: Implemented unrectifyPoint, with unit tests.
* image_geometry: Fixed bug in rectifyPoint due to cv::undistortPoints not accepting double pt data, `#4053 <https://github.com/ros-perception/vision_opencv/issues/4053>`_.
* image_geometry: Tweaked manifest.
* image_geometry: Better manifest description.
* Removed tfFrame sample
* image_geometry: Doxygen main page, manifest updates.
* image_geometry: Doxygen for StereoCameraModel.
* image_geometry: Made Q calculation match old stereoproc one.
* image_geometry: Tweaked projectDisparityImageTo3D API for handling missing values.
* image_geometry: Added method to project disparity image to 3d. Added ConstPtr version of fromCameraInfo in StereoCameraModel.
* image_geometry: Export linker flags. Fixed bug that could cause rectification maps to not be initialized before use.
* Fixed path-handling on gtest for CMake 2.6
* image_geometry: Added missing source file.
* image_geometry: Added some C++ docs.
* image_geometry: Minor cleanup of StereoCameraModel, added it to build. Put in copy constructors.
* image_geometry: Switched pinhole_camera_model to use new C++ OpenCV types and functions.
* Remove use of deprecated rosbuild macros
* image_geometry (C++): Unit test for projecting points uv <-> xyz.
* image_geometry (C++): Implemented more projection functions, added beginnings of the unit tests.
* trigger rebuild
* Enable rosdoc.yaml
* docs
* image_geometry: Started C++ API. PinholeCameraModel is in theory (untested) able to track state efficiently and rectify images.
* First stereo test
* Checkpoint
* Skeleton of test
* First cut
* Contributors: Vincent Rabaud, ethanrublee, gerkey, jamesb, mihelich, vrabaud, wheeler
