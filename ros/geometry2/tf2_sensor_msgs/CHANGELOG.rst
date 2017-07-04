^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tf2_sensor_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.13 (2016-03-04)
-------------------
* add missing Python runtime dependency
* fix wrong comment
* Adding tests to package
* Fixing do_transform_cloud for python
  The previous code was not used at all (it was a mistake in the __init_\_.py so
  the do_transform_cloud was not available to the python users).
  The python code need some little correction (e.g there is no method named
  read_cloud but it's read_points for instance, and as we are in python we can't
  use the same trick as in c++ when we got an immutable)
* Contributors: Laurent GEORGE, Vincent Rabaud

0.5.12 (2015-08-05)
-------------------

0.5.11 (2015-04-22)
-------------------

0.5.10 (2015-04-21)
-------------------

0.5.9 (2015-03-25)
------------------

0.5.8 (2015-03-17)
------------------
* ODR violation fixes and more conversions
* Fix keeping original pointcloud header in transformed pointcloud
* Contributors: Paul Bovbel, Tully Foote, Vincent Rabaud

0.5.7 (2014-12-23)
------------------
* add support for transforming sensor_msgs::PointCloud2
* Contributors: Vincent Rabaud
