.. _building_libraries_2:

Building and installing C++ libraries and headers
-------------------------------------------------

In catkin, libraries will be installed in a common directory shared by
all the packages in that entire ROS distribution.  So, make sure your
library names are sufficiently unique not to clash with other packages
or system libraries.  It makes sense to include at least part of your
package name in each library target name.

For this example, suppose ``your_package`` has a shared library build
target named ``your_library``.  On Linux, the actual file name will be
something like ``libyour_library.so``, perhaps with a version number
suffix.

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

Other :ref:`how-to pages <how_to_do_common_tasks_2>` describe how to
resolve header dependencies in more detail.

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

Exporting
:::::::::

Your ``catkin_package()`` needs to export all your library build
targets so other catkin packages can use them.  Suppose
``your_library`` depends on the ROS ``std_msgs`` package and on the
system ``Boost`` thread library::

  catkin_package(CATKIN_DEPENDS std_msgs
                 DEPENDS Boost
                 INCLUDE_DIRS include
                 LIBRARIES your_library)

Be sure to list every library on which your libraries depend, and
don't forget to mention them in your ``package.xml`` using a
``<depend>`` or ``<build_export_depend>`` tag::

  <depend>std_msgs</depend>
  <build_export_depend>boost</build_export_depend>

Installing
::::::::::

In catkin, libraries are installed in a ``lib/`` directory shared by
all the packages in that entire ROS distribution.  So, be careful what
you name them.

Add this command to your ``CMakeLists.txt``, mentioning all your
library build targets::

  install(TARGETS your_library your_other_library
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
