.. _installing_cmake_2:

Installing CMake files
----------------------

Sometimes your package needs to install CMake files for use by other
packages.  For that to work, your ``catkin_package()`` command must
include a ``CFG_EXTRAS`` parameter, where you list the CMake files you
want to export::

  catkin_package(CFG_EXTRAS your_macros.cmake your_modules.cmake)

When another package uses ``find_package()`` on this package, the listed
CMake files are automatically included.

Since these data are platform-independent, they should be installed in
your package's **share/** subtree.  This example assumes your CMake
sources are in the customary **cmake/** subdirectory::

  install(FILES cmake/your_macros.cmake cmake/your_modules.cmake
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cmake)

To install everything in your **cmake/** subdirectory::

  install(DIRECTORY cmake
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
          PATTERN ".svn" EXCLUDE)

The ``PATTERN ".svn" EXCLUDE`` is only needed if you use a Subversion_
repository.  For other types of repositories, it can be omitted.

.. _`roslaunch scripts`: http://wiki.ros.org/roslaunch/XML
.. _Subversion: http://subversion.apache.org/
