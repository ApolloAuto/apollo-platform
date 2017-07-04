^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pluginlib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.5 (2017-03-27)
-------------------
* Merge pull request `#47 <https://github.com/ros/pluginlib/issues/47>`_ from ros/fix_conversion
  fix size_t to int conversion
* fix int conversion
* Contributors: Mikael Arguedas

1.10.4 (2016-09-20)
-------------------
* Merge pull request `#42 <https://github.com/ros/pluginlib/issues/42>`_ from delftrobotics-forks/unique-ptr
  Add std::unique_ptr API
* Add unit test for unique_ptr API.
* Simplify unit tests with ASSERT_THROW.
* Add ClassLoader::createUniqueInstance.
* Wrap long comment on createInstance and friend.
* Throw exception if plugin.xml is broken (`#41 <https://github.com/ros/pluginlib/issues/41>`_)
  * added test case for broken xml files with missing attributes of class tag
  * added checks if all needed attributes of the class tag are existing
  * removed comment and empty line
* Contributors: Maarten de Vries, Mikael Arguedas, cwecht

1.10.3 (2016-06-22)
-------------------
* Merge pull request `#40 <https://github.com/ros/pluginlib/issues/40>`_ from ros/fix_warnings
  fix deprecated warnings in unit tests
* fix deprecated warnings in unit tests
* removed merge messages and redundant commits
* Contributors: Mikael Arguedas

1.10.2 (2016-03-14)
-------------------
* Remove Boost Software License from license tag `#35 <https://github.com/ros/pluginlib/issues/35>`_
* Throw an exception if ClassLoader can't be instantiated due to an invalid package name `#34 <https://github.com/ros/pluginlib/issues/34>`_
* Add ":" to split function within getName. `#33 <https://github.com/ros/pluginlib/issues/33>`_
* Contributors: Esteve Fernandez, Jochen Sprickerhof, Mikael Arguedas, Mike O'Driscoll

1.10.1 (2014-12-23)
-------------------
* Remove GTEST_FOUND from CMakeLists.txt
* Check that GTest is installed before running tests.
* Moved plugin_macro_update script to scripts directory. Made plugin_macro_update rosrunnable and removed it from global PATH `#29 <https://github.com/ros/pluginlib/issues/29>`_
* Contributors: Esteve Fernandez

1.10.0 (2014-05-08 14:56)
-------------------------

1.9.25 (2014-05-08 20:37)
-------------------------
* Use cmake_modules to find TinyXML `#26 <https://github.com/ros/pluginlib/issues/26>`_
* Check for release libraries in debug builds `#25 <https://github.com/ros/pluginlib/issues/25>`_
* update refreshDeclaredClasses to force recrawl (fix `#23 <https://github.com/ros/pluginlib/issues/23>`_)
* Contributors: Dirk Thomas, Esteve Fernandez

1.9.24 (2014-03-11)
-------------------
* Remove invalid exception when no plugins are found `#22 <https://github.com/ros/pluginlib/issues/22>`_
* Update maintainer field
* Contributors: Dirk Thomas, Esteve Fernandez

1.9.23 (2013-10-04)
-------------------
* expose plugin paths in ClassLoader `#21 <https://github.com/ros/pluginlib/issues/21>`_
* Contributors: Dirk Thomas, Mirza Shah

1.9.22 (2013-08-21)
-------------------
* Fixed use of __FILE_\_ macro in deprecation warning
* Added libdl to plugin_tool link args...temporary fix
* Contributors: Mirza Shah

1.9.21 (2013-07-14)
-------------------
* Added file hint for deprecated warnings. `#16 <https://github.com/ros/pluginlib/issues/16>`_
* check for CATKIN_ENABLE_TESTING
* remove mainpage.dox
* Contributors: Dane Powell, Dirk Thomas, Mirza Shah

1.9.20 (2013-04-18)
-------------------
* Added another unit test for managed instance case.
* Fixed a regression that broke unload call. Added a unit test for this case.
* Contributors: Mirza Shah

1.9.19 (2013-03-23)
-------------------
* Converted ROS_DEBUG and ROS_WARN calls to ROS_DEBUG_NAMED and ROS_WARN_NAMED calls `#13 <https://github.com/ros/pluginlib/issues/13>`_
* Contributors: Dave Coleman, Mirza Shah

1.9.18 (2013-01-28)
-------------------
* Support for boost filesystem v2 `#11 <https://github.com/ros/pluginlib/issues/11>`_
* Added more debug information
* Contributors: Mario Prats, Mirza Shah

1.9.17 (2012-12-27)
-------------------
* More useful debug messages
* Fixed incorrect debug message in plugin description XML parsing
* Contributors: Mirza Shah

1.9.16 (2012-12-21)
-------------------
* Removed old file
* Annotated deprecation warning with more info
* Made python script global installable
* Added a script to recursively update deprecated pluginlib macro
* added missing license header
* modified dep type of catkin
* Contributors: Aaron Blasdel, Dirk Thomas, Mirza Shah

1.9.15 (2012-12-13 17:22)
-------------------------
* Updated registration macros to be easier and deprecated older ones. Also cleaned up code violating standard
* Added wg copyright notice
* Contributors: Mirza Shah

1.9.14 (2012-12-13 15:20)
-------------------------
* lookup name (i.e. magic name) is now optional. Further cleanup...alphabetized methods, broke up some.
* Contributors: Mirza Shah

1.9.13 (2012-12-11)
-------------------
* Made robust to plugin package having different name from the folder it came from. ```#6 <https://github.com/ros/pluginlib/issues/6`_``
* Contributors: Mirza Shah

1.9.12 (2012-12-06)
-------------------
* Cleaned up debug output a little more
* Contributors: Mirza Shah

1.9.11 (2012-11-26)
-------------------
* Fixed a regression that somehow got back in there that was causing a race condition in multithreaded code, this will fix gazebo issues
* Bug fixes
* Contributors: Mirza Shah, mirzashah

1.9.10 (2012-11-21)
-------------------
* Created plugintool
* Contributors: Mirza Shah

1.9.9 (2012-11-16)
------------------
* Minor fix where library was being unloaded for old load/unload reference counting, not needed anymore as class_loader handles that
* Contributors: Mirza Shah

1.9.8 (2012-11-14)
------------------
* refactored to return reasonable library path before loading the library
* Updated registration macros to correct legacy PLUGINLIB_REGISTER_CLASS macro as well as cleaned up comments
* Contributors: Dirk Thomas, Mirza Shah

1.9.7 (2012-11-08)
------------------
* updated catkin_package(DEPENDS)
* add missing Boost_INCLUDE_DIRS
* Contributors: Dirk Thomas

1.9.6 (2012-11-07)
------------------
* Added more debug messages and fixed a bug where managed instances do not auto open library
* Contributors: Mirza Shah

1.9.5 (2012-11-06)
------------------
* Changed ROS_ERROR to ROS_DEBUG
* Contributors: Mirza Shah

1.9.4 (2012-11-05)
------------------
* Removed more cruft and made pluginlib header only
* Removed unnecessary boost_fs_wrapper target, pluginlib now purely header only
* Made error message more meaningful
* Contributors: Mirza Shah

1.9.3 (2012-10-31)
------------------
* Fix to check for package.xml and not just manifest.xml when trying to verify a package. `#1 <https://github.com/ros/pluginlib/issues/1>`_
* Contributors: Mirza Shah

1.9.2 (2012-10-25)
------------------
* fixed deps for downstream packages
* Contributors: Dirk Thomas

1.9.1 (2012-10-24 22:02)
------------------------
* fix missing and redundant deps for downstream projects
* Contributors: Dirk Thomas

1.9.0 (2012-10-24 18:31)
------------------------
* renamed test target
* remove obsolete files
* Fixed dependency in package.xml and minor touchups
* Broke up code into further files
* Catkinized pluginlib and completed integration more or less with class_loader. Heavy mods to pluginlib::ClassLoader to handle constraints of Catkin as well as delegate housekeeping to class_loader::ClassLoader
* Updated to utilize newly renamed class_loader (formerly plugins) library with new file names, functions, identifiers, etc
* Removed explicit dependency that should have been automatically imported from dependent package in CMakeLists.txt
* Fixed unhandled exception to make all unit tests pass
* Removed mention of console bridge in CMakeLists.txt, plugins now probably exports
* Finished mods to utilize lower level plugins library. One test still failing, will get to that soon, but basics seem to be ok
* Modding pluginlib to use new plugins library. Not done, but just doing it tosync with my laptop
* Removed Poco and updated CMake and manifest files to depend on lower level plugins library
* Contributors: Dirk Thomas, Mirza Shah, mirzashah

1.8.6 (2012-10-09)
------------------
* added missing boost include dirs and runtime dependency
* updated cmake min version to 2.8.3
* Contributors: Dirk Thomas, Vincent Rabaud

1.8.5 (2012-10-01)
------------------
* add missing roslib dependency that happens in class_loader_imp.h
* Contributors: Vincent Rabaud

1.8.4 (2012-09-30)
------------------
* updated to latest catkin
* Contributors: Dirk Thomas

1.8.3 (2012-09-07)
------------------
* added tinyxml to project depends
* Contributors: Dirk Thomas

1.8.2 (2012-09-06)
------------------
* updated pkg-config in manifest.xml
* updated catkin variables
* Contributors: Dirk Thomas

1.8.1 (2012-09-04)
------------------
* Missing LIBRARIES and DEPENDS specifiers from CMakeLists.txt, now added.
* catkin-ized
* updated api doc for load/create/unload methods
* renamed new methods using shorter name for encouraged method
* added cmake macro for hiding plugin symbols and respective rosbuild export
* updated class loader according to updated REP 121
* add auto-unload for libraries using boost shared pointer
* pluginlib: added a pure-virtual base class for ClassLoader called ClassLoaderBase, which is not templated.  Only one function of ClassLoader is actually templated.  This allows client code to not be templated where it doesn't need to be.
* patch 4 for `#4887 <https://github.com/ros/pluginlib/issues/4887>`_
* ignore bin
* accepting patch from ticket `#4887 <https://github.com/ros/pluginlib/issues/4887>`_ REP 116 implementation
* add explicit link against tinyxml, because users of our libraries will need to link against it
* link poco_lite with tinyxml
* remove namespace to be compatible with tinyxml sysdep
* removing back depend on common
* removing rosdep.yaml, rule is in ros/rosdep.yaml
* fixed tinyxml
* converting to unary stack (separated from common)
* applied patch from 4923, to support boost 1.46
* patch from Nick Butko osx compatability
* adding unittest melonee forgot to commit
* adding pluginlib tests
* patch for osx linking `#4094 <https://github.com/ros/pluginlib/issues/4094>`_
* Fixed exception comments
* Added Ubuntu platform tags to manifest
* Fixing bug where the incorrect library path was passed to dlopen from pluginlib... oops.
* fix in latest for `#4013 <https://github.com/ros/pluginlib/issues/4013>`_ to isolate boost filesystem calls into a library
* patch from Wim `#3346 <https://github.com/ros/pluginlib/issues/3346>`_ reviewed by Eitan and I
* Adding getName and isClassAvailable function calls to the class loader
* inlining to avoid multiple definitions
* macro deprecation
* adding warning about deprecated macro PLUGINLIB_REGISTER_CLASS
* pluginlib now takes pkg/type arguments, new macro PLUGINLIB_DECLARE_CLASS
* pluginlib now robust to malformed manifests
* Adding more descriptive error messages when libaries fail to load
* Remove use of deprecated rosbuild macros
* doc review completed http://www.ros.org/wiki/pluginlib/Reviews/2009-10-06_Doc_Review
* fixing documentation link
* fixing `#2894 <https://github.com/ros/pluginlib/issues/2894>`_
* Removing ROS_ERRORS in favor of adding information to the exceptions thrown
* migration part 1
* Contributors: Dave Hershberger, Dirk Thomas, Ken Conley, Mirza Shah, Tully Foote, eitan, gerkey, kwc, mwise, rusu, tfoote, vpradeep, wheeler
