.. _find_package_internals:

Finding required packages
=========================

Recommended method
------------------

If you want to specify a dependency on several catkin components
simultaneously, use
``find_package(catkin [XX.YY] REQUIRED COMPONENTS comp1 comp2)``, e.g.::

  find_package(catkin REQUIRED COMPONENTS
               cpp_common rostime roscpp_traits
               roscpp_serialization sensor_msgs)
  include_directories(${catkin_INCLUDE_DIRS})

  add_executable(myexec ...)
  target_link_libraries(myexec ${catkin_LIBRARIES})

You can also reference the variables of each component individually::

  target_link_libraries(myexec ${rostime_LIBRARIES})

See the CMake documentation for ``find_package()`` for more details.
Your ``CMAKE_PREFIX_PATH`` will need to point to a catkin installation.


find_package() config mode
--------------------------

CMake's ``find_package`` is the preferred method for packages to
communicate to CMake (and thereby to catkin) the libraries, include
directories and such that packages should use.

There are a couple of modes of operation of find_package (see the
CMake documentation), "module" mode and "config" mode.  "module" mode
is the one that uses CMake scripts named ``Find****.cmake``.  "config"
mode is the preferred mode, and it works differently.

One reason we find the 'config mode' superior is that is supports
multiple simultaneous installed versions of packages.

For a package ``t``, 'config mode' consists of two CMake files, both of
which are installed somewhere on CMake's ``CMAKE_PREFIX_PATH``.  The
first is 'tConfig-version.cmake'.  We find the most succinct form to
be like this::

  set(PACKAGE_VERSION_EXACT False)
  set(PACKAGE_VERSION_COMPATIBLE False)
  if("${PACKAGE_FIND_VERSION}" STREQUAL "")
    set(PACKAGE_VERSION_COMPATIBLE TRUE)
    return()
  endif()

  if("${PACKAGE_FIND_VERSION}" VERSION_EQUAL "9.9.9")
    set(PACKAGE_VERSION_EXACT True)
    set(PACKAGE_VERSION_COMPATIBLE True)
  endif()

  if("${PACKAGE_FIND_VERSION}" VERSION_LESS "9.9.9")
    set(PACKAGE_VERSION_COMPATIBLE True)
  endif()

where `9.9.9` is replaced by the numeric version of the package.  The
second file, ``tConfig.cmake``, tells the client what the assorted
includes/libs are for the package by setting variables
``t_INCLUDE_DIRS``, ``t_LIBRARIES``, ``t_LIBRARY_DIRS`` and so forth.
The user passes these values to CMake's ``include_directories()`` and
``target_link_libraries()``. Since the libraries contains absolute
paths ``link_directories()`` is not necessary.
