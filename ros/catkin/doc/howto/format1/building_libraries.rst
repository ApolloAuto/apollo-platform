.. _building_libraries_1:

Building and installing C++ libraries and headers
-------------------------------------------------

In catkin, libraries will be installed in a common directory shared by
all the packages in that entire ROS distribution.  So, make sure your
library names are sufficiently unique not to clash with other packages
or system libraries.

For this example, suppose you have a shared library build target named
``your_library``.  On Linux, the actual file name will be something
like ``libyour_library.so``, perhaps with a version number suffix.

Headers
:::::::

Before compiling, collect all the header paths for your build
dependencies::

  include_directories(include
                      ${catkin_INCLUDE_DIRS}
                      ${Boost_INCLUDE_DIRS}
                      ${GSTREAMER_INCLUDE_DIRS})

That only needs to be done once in your ``CMakeLists.txt``.  These
parameters are just examples.  The ``include`` parameter is needed
only if that subdirectory of your source package contains headers used
to compile your programs.  All your catkin package header dependencies
are resolved via ``${catkin_INCLUDE_DIRS}``.

Other :ref:`how_to_do_common_tasks_1` pages describe how to resolve
header dependencies in more detail.

Building
::::::::

To build your library add this command to your ``CMakeLists.txt``,
listing all required C++ source files, but not the headers::

  add_library(your_library libsrc1.cpp libsrc2.cpp libsrc_etc.cpp)

If the list of source files is long, a CMake variable can help::

  set(YOUR_LIB_SOURCES
      libsrc1.cpp
      libsrc2.cpp
      libsrc3.cpp
      libsrc4.cpp
      libsrc_etc.cpp)
  add_library(your_library ${YOUR_LIB_SOURCES})

If your library depends on libraries provided by other catkin
packages, add this command::

  target_link_libraries(your_library ${catkin_LIBRARIES})

If your library depends on additional non-catkin system libraries,
include them in the ``target_link_libraries()``::

  target_link_libraries(your_library
                        ${catkin_LIBRARIES}
                        ${Boost_LIBRARIES}
                        ${GSTREAMER_LIBRARIES})

If the list of libraries is lengthy, you can similarly define a CMake
variable for them.

Installing
::::::::::

In catkin, libraries are installed in a ``lib/`` directory shared by
all the packages in that entire ROS distribution.  So, be careful what
you name them.

Add these commands to your ``CMakeLists.txt``, substituting all your
library build target names for ``your_library``::

  catkin_package(INCLUDE_DIRS include
                 LIBRARIES your_library)

  install(TARGETS your_library
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

The runtime destination is used for `.dll` file on Windows which must
be placed in the global bin folder.

Libraries typically provide headers defining their interfaces.  Please
follow standard ROS practice and place all external header files under
``include/your_package/``::

  install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

That command installs all the files in your package's include subtree.
Place only your exported headers there.  If yours is a Subversion_
repository, don't forget to exclude the ``.svn`` subdirectories like
this::

  install(DIRECTORY include/${PROJECT_NAME}/
          DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
          PATTERN ".svn" EXCLUDE)

.. _Subversion: http://subversion.apache.org/
