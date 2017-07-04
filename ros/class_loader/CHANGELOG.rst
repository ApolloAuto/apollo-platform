^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package class_loader
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.6 (2016-10-24)
------------------
* Made changes to two locking mechanisms inside class loader core's loadLibrary() function: 1) I added a lock to the 'addClassLoaderOwnerFor...' function to protect it against a race condition with the unloadLibrary() function. 2) I also raised the loader lock to cover the whole function. Previously the check to see if a library is already loaded and the actual loading of the library was not atomic. Multiple threads could create shared library objects, for example.
* Contributors: Jonathan Meyer

0.3.5 (2016-09-20)
------------------
* Add ClassLoader::createUniqueInstance (`#38 <https://github.com/ros/class_loader/issues/38>`_)
  * Wrap comments on createInstance and friend.
  * Delegate createInstance and createUnmanagedInstance to private impl.
  * Add ClassLoader::createUniqueInstance.
  * MultiLibraryClassLoader: Factor out getClassLoaderForClass.
  * MultiLibraryClassLoader: Add unique_ptr API.
  * Add tests for unique_ptr API.
* Contributors: Maarten de Vries

0.3.4 (2016-06-22)
------------------
* cleanup: don't use active_class_loaders\_[library_path] for existence test (`#35 <https://github.com/ros/class_loader/issues/35>`_)
  * cleanup: don't use active_class_loaders\_[library_path] for existence test
  - this accumulates map entries with NULL pointer
  - fixing it, allows some cleanup
  * list headers in CodeBlocks / QtCreator
  * explicitly list all headers
* Merge pull request `#34 <https://github.com/ros/class_loader/issues/34>`_ from rhaschke/fix-on-demand-unloading
  fix on demand unloading
* Merge pull request `#32 <https://github.com/ros/class_loader/issues/32>`_ from saarnold/fixed_unset_variable_evaluation
  fixed evaluation of undefined variable
* fixed evaluation of undefined variable
* not unloading the ClassLoaders (to avoid the SEVERE WARNING) doesn't work either
* bugfix: enable on-demand loading/unloading with MultiClassLoader
  - enforce loading of library in loadLibrary(), otherwise we cannot know
  - don't unload libraries in destructor when on-demand-unloading is enabled
* extra utest: MultiClassLoaderTest.lazyLoad succeeds two times in a row?
* added MultiLibraryClassLoader unittest
* Contributors: Mikael Arguedas, Robert Haschke, Sascha Arnold

0.3.3 (2016-03-10)
------------------
* update maintainer
* Merge pull request `#26 <https://github.com/ros/class_loader/issues/26>`_ from goldhoorn/indigo-devel
  Added option to disable the catkin build
* Added option to disable the catkin build
* Contributors: Esteve Fernandez, Matthias Goldhoorn, Mikael Arguedas

0.3.2 (2015-04-22)
------------------
* Fixed wrong handling of false statement (pkg-config was not installed)
* Make catkin optional again
* Contributors: Esteve Fernandez, Janosch Machowinski, Matthias Goldhoorn

0.3.1 (2014-12-23)
------------------
* Depend on boost
* Use FindPoco.cmake from ros/cmake_modules
*  Honor BUILD_SHARED_LIBS and do not force building shared libraries.
* Contributors: Esteve Fernandez, Gary Servin, Scott K Logan

0.3.0 (2014-06-25)
------------------
* Use system-provided console-bridge
* Contributors: Esteve Fernandez

0.2.5 (2014-03-04)
------------------
* Changed format of debug messages so that rosconsole_bridge can correctly parse the prefix
* Improved debug output

0.2.4 (2014-02-12)
------------------
* fix race condition with multi threaded library loading (`#16 <https://github.com/ros/class_loader/issues/16>`_)

0.2.3 (2013-08-21)
------------------
* fix missing class name in logWarn output

0.2.2 (2013-07-14)
------------------
* check for CATKIN_ENABLE_TESTING (`#10 <https://github.com/ros/class_loader/issues/10>`_)
* fix find Poco to return full lib path (`#8 <https://github.com/ros/class_loader/issues/8>`_)
* add missing runtime destination for library under Windows
* add Boosst component system

0.2.1 (2013-06-06)
------------------
* improve check for Poco foundation and headers (`#7 <https://github.com/ros/class_loader/issues/7>`_)

0.2.0 (2013-03-13)
------------------
* use find_package for Poco/dl instead to make it work on other platforms
* update Poco cmake file to include libdl on non-windows systems
* No longer CATKIN_DEPEND on console_bridge

0.1.27 (2013-01-25)
-------------------
* change warning message for managed/unmanaged instance mixture in lazy loading mode

0.1.26 (2013-01-17)
-------------------
* fix all instances marked as unmanaged

0.1.25 (2013-01-16)
-------------------
* fix redundant destructor definition being pulled into plugin library for metaobjects instead of being contained with libclass_loader.so

0.1.24 (2013-01-14 15:27)
-------------------------
* fix syntax error for logInform

0.1.23 (2013-01-14 15:23)
-------------------------
* downgrade some warning messages to be info/debug

0.1.22 (2013-01-14 15:01)
-------------------------
* add safety checks for mixing of managed/unmanaged mixing as well as pointer equivalency check between graveyard and newly created metaobjects

0.1.21 (2013-01-13)
-------------------
* fix compile issue on OSX in dependent packages (`#3 <https://github.com/ros/class_loader/issues/3>`_)
* add more debug information

0.1.20 (2012-12-21 16:04)
-------------------------
* first public release for Groovy
