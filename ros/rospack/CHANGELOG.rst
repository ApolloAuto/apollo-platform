^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rospack
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.2.8 (2017-03-06)
------------------
* make some deps* functions public (`#65 <https://github.com/ros/rospack/pull/65>`_)

2.2.7 (2016-06-27)
------------------
* allow caching of rospack results (`#49 <https://github.com/ros/rospack/issues/49>`_)

2.2.6 (2016-03-09)
------------------
* fix memory leak in Rosstackage::addStackage (`#59 <https://github.com/ros/rospack/issues/59>`_)
* return false in depsOnDetail if the package name in rospack plugins can not be found (`#51 <https://github.com/ros/rospack/issues/51>`_)
* #undef symbols before #defining them to avoid preprocessor warnings in the case that they were already #defined (`#50 <https://github.com/ros/rospack/issues/50>`_)

2.2.5 (2014-09-04)
------------------
* support tags defined in package format 2 (`#43 <https://github.com/ros/rospack/issues/43>`_)

2.2.4 (2014-07-10)
------------------
* fix find_package(PythonLibs ...) with CMake 3 (`#42 <https://github.com/ros/rospack/issues/42>`_)

2.2.3 (2014-05-07)
------------------
* find library for exact Python version (even if not in CMake provided list of version numbers) (`#40 <https://github.com/ros/rospack/issues/40>`_)
* find TinyXML using cmake_modules (`#24 <https://github.com/ros/rospack/issues/24>`_)
* make error messages tool specific (rospack vs. rosstack) (`#38 <https://github.com/ros/rospack/issues/38>`_)

2.2.2 (2014-02-25)
------------------
* python 3 compatibility (`#35 <https://github.com/ros/rospack/issues/35>`_)

2.2.1 (2014-02-24)
------------------
* only perform backquote substitution when needed (`#34 <https://github.com/ros/rospack/issues/34>`_)

2.2.0 (2014-01-30)
------------------
* add hash of ROS_PACKAGE_PATH to rospack/rosstack cache filename, remove ROS_ROOT from cache (`#28 <https://github.com/ros/rospack/issues/28>`_)

2.1.22 (2014-01-07)
-------------------
* use specific python version catkin has decided on (`#29 <https://github.com/ros/rospack/issues/29>`_)
* python 3 compatibility (`#25 <https://github.com/ros/rospack/issues/25>`_, `#27 <https://github.com/ros/rospack/issues/27>`_)
* fall back gracefully whe gtest is not available
* update package urls

2.1.21 (2013-07-05)
-------------------
* honor CATKIN_IGNORE marker file when crawling for packages (`#21 <https://github.com/ros/rospack/issues/21>`_)

2.1.20 (2013-07-03)
-------------------
* improve error message to include package names when circular dependency is detected (`#18 <https://github.com/ros/rospack/issues/18>`_)
* check for CATKIN_ENABLE_TESTING to enable configure without tests
* add '-h' option

2.1.19 (2013-06-06)
-------------------
* modified command 'list-duplicates' to output the paths where the packages were found (`#3 <https://github.com/ros/rospack/issues/3>`_)
* modified 'rospack plugins' to not use rosdep (`#5 <https://github.com/ros/rospack/issues/5>`_)
* improve Windows support  (`#10 <https://github.com/ros/rospack/issues/10>`_)
* use find_package() for tinyxml (if available)

2.1.18 (2013-03-21)
-------------------
* invert order of package type detection (dry before wet) (`ros-infrastructure/rospkg#30 <https://github.com/ros-infrastructure/rospkg/issues/30>`_)

2.1.17 (2013-03-08)
-------------------
* output full pkg-config command in case of errors (`#8 <https://github.com/ros/rospack/issues/8>`_)
* handle None as return value for call_pkg_config (`#8 <https://github.com/ros/rospack/issues/8>`_)
* fix crawling to always recrawl when forced (`#9 <https://github.com/ros/rospack/issues/9>`_)

2.1.16 (2013-01-13)
-------------------
* fix segfault for command depends1 which ignores exceptions and calls isSysPackage again (`#4 <https://github.com/ros/rospack/issues/4>`_)

2.1.15 (2012-12-06)
-------------------
* first public release for Groovy
