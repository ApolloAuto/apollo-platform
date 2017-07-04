.. _how_to_do_common_tasks_1:

Package format 1 (legacy)
=========================

When writing a ROS package several tasks often need to be done.  These
pages give examples of how to handle most of the common ones.

.. note::

   These instructions are for the original ``<package format="1">``.

   If you are making small changes to a format 1 package, use these
   instructions.  If you are ready to upgrade your package to format
   2, see: :ref:`migrating_from_format1_to_format2`.

Overview
--------

.. toctree::
   :maxdepth: 1

   catkin_overview

Resolving dependencies
----------------------

Packages almost always use features provided by other packages.
Describe all your direct dependencies.  Their transitive dependencies
on other packages are handled automatically by catkin.

.. toctree::
   :maxdepth: 1

   catkin_library_dependencies
   system_library_dependencies
   cpp_msg_dependencies
   python_module_dependencies


Building and installing targets
-------------------------------

*Build targets* are generated binaries, shared libraries, message
headers, and other objects.  Various targets require special handling.

.. toctree::
   :maxdepth: 1

   building_executables
   building_libraries
   building_msgs
   dynamic_reconfiguration
   installing_python
   installing_cmake
   installing_other


Configuring and running unit tests
----------------------------------

All configuration steps related to testing should be only done
conditionally if ``CATKIN_ENABLE_TESTING`` is set (which is true by
default).  Passing ``-DCATKIN_ENABLE_TESTING=0`` to CMake enables to
configure and build packages without any testing overhead.

If the CMake configuration of packages fails because they use test
functions without checking this flag before the option
``-DCATKIN_SKIP_TESTING=1`` can be passed to CMake to disable testing
but providing mock functions in order to not break the build.

.. toctree::
   :maxdepth: 1

   downloading_test_data
   gtest_configuration
   python_nose_configuration
   rostest_configuration
   run_tests
