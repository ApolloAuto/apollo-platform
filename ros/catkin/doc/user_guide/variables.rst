.. _variables:

Variables
=========

Build space
-----------

.. cmake:data:: CATKIN_DEVEL_PREFIX

   This is set to ``${CMAKE_BINARY_DIR}/devel`` by ``catkin/all.cmake`` and is the analogue to ``CMAKE_PREFIX_PATH``.
   Since the layout of the both folders ``CATKIN_DEVEL_PREFIX`` and ``CMAKE_PREFIX_PATH`` is identical you can append any of the following install destinations to the build prefix.

Install destinations
--------------------

All destination variables are meant to be used with the CMake ``install()`` macro as an ``DESTINATION`` argument.
They only contain relative paths and are supposed to be relative to the ``${CMAKE_INSTALL_PREFIX}`` (or ``${CATKIN_DEVEL_PREFIX}``).

**Project specific**

.. cmake:data:: CATKIN_PACKAGE_BIN_DESTINATION

   This is set to ``${CATKIN_GLOBAL_LIBEXEC_DESTINATION}/${PROJECT_NAME}``.

.. cmake:data:: CATKIN_PACKAGE_ETC_DESTINATION

   This is set to ``${CATKIN_GLOBAL_ETC_DESTINATION}/${PROJECT_NAME}``.

.. cmake:data:: CATKIN_PACKAGE_INCLUDE_DESTINATION

   This is set to ``${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}``.

.. cmake:data:: CATKIN_PACKAGE_LIB_DESTINATION

   This is set to ``${CATKIN_GLOBAL_LIB_DESTINATION}``.
   All libraries go into a global folder.  Still use this variable instead of ``CATKIN_GLOBAL_LIB_DESTINATION`` for package libraries.

.. cmake:data:: CATKIN_PACKAGE_PYTHON_DESTINATION

   This is set to ``${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME}``.

.. cmake:data:: CATKIN_PACKAGE_SHARE_DESTINATION

   This is set to ``${CATKIN_GLOBAL_SHARE_DESTINATION}/${PROJECT_NAME}``.

**Global**

.. cmake:data:: CATKIN_GLOBAL_BIN_DESTINATION

   This is set to ``bin``.
   This destination should only be used for the core ROS binaries.
   If you are unsure if you should use this destination use ``CATKIN_PACKAGE_LIBEXEC_DESTINATION`` instead.

.. cmake:data:: CATKIN_GLOBAL_ETC_DESTINATION

   This is set to ``etc``.

.. cmake:data:: CATKIN_GLOBAL_INCLUDE_DESTINATION

   This is set to ``include``.
   This destination should only be used when installing headers which are not in a subfolder which matches the project name.
   Else you should use the destination ``CATKIN_PACKAGE_INCLUDE_DESTINATION`` instead.

.. cmake:data:: CATKIN_GLOBAL_LIB_DESTINATION

   This is set to ``lib``.
   This variable should not be used directly, use ``CATKIN_PACKAGE_LIB_DESTINATION`` instead.

.. cmake:data:: CATKIN_GLOBAL_LIBEXEC_DESTINATION

   This is set to ``lib``.
   On non-Debian distributions it could be set to ``libexec``.
   Note that setting this variable to anything but ``lib`` or ``libexec`` will
   not work as expected since tools like ``catkin_find`` will only consider
   these two locations.
   This variable should not be used directly, use ``CATKIN_PACKAGE_BIN_DESTINATION`` instead.

.. cmake:data:: CATKIN_GLOBAL_PYTHON_DESTINATION

   This is set to ``lib/pythonX.Y/dist-packages`` (Debian), ``lib/pythonX.Y/site-packages`` (non-Debian) or ``lib/site-packages`` (Windows).

.. cmake:data:: CATKIN_GLOBAL_SHARE_DESTINATION

   This is set to ``share``.

Environment
-----------

.. cmake:data:: CATKIN_ENV

   The path to the shell script ``env.sh`` that will execute its
   arguments inside the catkin environment.  CMake that executes shell
   commands (e.g. as part of ``add_custom_command``) should use this
   rather than wrangling environment explicitly.
