.. _building_executables_2:

Building and installing C++ executables
---------------------------------------

For this example, suppose you have an executable build target named
``your_node``.

Headers
:::::::

Before compiling, collect all the header paths for your build
dependencies::

  include_directories(include
                      ${catkin_INCLUDE_DIRS}
                      ${Boost_INCLUDE_DIRS}
                      ${GSTREAMER_INCLUDE_DIRS})

These parameters are just examples.  The ``include`` parameter is
needed only if that subdirectory of your source package contains
headers used to compile your programs.  All your catkin package header
dependencies are resolved via ``${catkin_INCLUDE_DIRS}``.

Other :ref:`how-to pages <how_to_do_common_tasks_2>` describe how to
resolve header dependencies in more detail.

Building
::::::::

To build ``your_node``, add this command to your ``CMakeLists.txt``,
listing all required C++ source files, but not the headers::

  add_executable(your_node src1.cpp src2.cpp src_etc.cpp)

If the list of files is long, a CMake variable can help::

  set(${PROJECT_NAME}_SOURCES
      src/file1.cpp
      src/file2.cpp
      src/file3.cpp
      src/file4.cpp
      src/file5.cpp
      src/file6.cpp
      src/file_etc.cpp)
  add_executable(your_node ${${PROJECT_NAME}_SOURCES})

If your program depends on libraries provided by other catkin
packages, add this::

  add_executable(your_node ${${PROJECT_NAME}_SOURCES})
  target_link_libraries(your_node ${catkin_LIBRARIES})

If your program depends on additional non-catkin system libraries,
include them in the ``target_link_libraries()``::

  add_executable(your_node ${${PROJECT_NAME}_SOURCES})
  target_link_libraries(your_node
                        ${catkin_LIBRARIES}
                        ${Boost_LIBRARIES}
                        ${GSTREAMER_LIBRARIES})

If the list of libraries is lengthy, you can similarly define a CMake
variable for them.

Installing
::::::::::

ROS executables are installed in a per-package directory, not the
distributions's global ``bin/`` directory.  There, they are accessible
to rosrun_ and roslaunch_, without cluttering up the shell's
``$PATH``, and their names only need to be unique within each package.
There are only a few core ROS commands like ``rosrun`` and
``roslaunch`` that install in the global ``bin/`` directory.

List all your executables as TARGETS on an install command like this::

  install(TARGETS your_node
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

.. _roslaunch: http://wiki.ros.org/roslaunch
.. _rosrun: http://wiki.ros.org/rosrun
