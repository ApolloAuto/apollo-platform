Extracted CMake API reference
=============================
This page was auto-generated from cmake source files using generate_cmake_rst.py

.. !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
.. !!!!!! Auto-generated file, do not modify
.. !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

.. contents::
   :local:


Public CMake functions / macros
-------------------------------

 * :cmake:macro:`catkin_add_env_hooks`
 * :cmake:macro:`catkin_add_executable_with_gtest`
 * :cmake:macro:`catkin_add_gtest`
 * :cmake:macro:`catkin_add_nosetests`
 * :cmake:macro:`catkin_download`
 * :cmake:macro:`catkin_download_test_data`
 * :cmake:macro:`catkin_filter_libraries_for_build_configuration`
 * :cmake:macro:`catkin_install_python`
 * :cmake:macro:`catkin_metapackage`
 * :cmake:macro:`catkin_pack_libraries_with_build_configuration`
 * :cmake:macro:`catkin_package`
 * :cmake:macro:`catkin_package_xml`
 * :cmake:macro:`catkin_python_setup`
 * :cmake:macro:`catkin_replace_imported_library_targets`
 * :cmake:macro:`catkin_unpack_libraries_with_build_configuration`

.. _`catkin_add_env_hooks_ref`:

`catkin_add_env_hooks`
~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_add_env_hooks(file_prefix)

 *[function defined in catkin_add_env_hooks.cmake]*


 Register environment hooks which are executed by the setup script.

 For each shell in ``SHELLS``, the macro searches for one of the
 following files in the directory ``DIRECTORY``:
 ``<file_prefix>.<shell>``,
 ``<file_prefix>.<shell>.<develspace|installspace>.em``,
 ``<file_prefix>.<shell>.em``,
 ``<file_prefix>.<shell>.<develspace|installspace>.in`` or
 ``<file_prefix>.<shell>.in``.

 Plain shells, will be copied to, templates are expanded to
 ``etc/catkin/profile.d/``, where it will be read by global generated
 ``setup.<shell>``.

 The templates can also distinguish between devel- and installspace
 using the boolean variables ``DEVELSPACE`` and ``INSTALLSPACE``
 which are either ``true`` or ``false``.
 E.g. @[if DEVELSPACE]@ ... @[end if]@ for .em

 .. note:: Note that the extra extensions must appear in the filename
   but must not appear in the argument.

 .. note:: These files will share a single directory with other
   packages that choose to install env hooks.  Be careful to give
   the file a unique name.  Typically ``NN.name.<shell>`` is used,
   where NN can define when something should be run (the files are
   read in alphanumeric order) and the name serves to disambiguate
   in the event of collisions.

 Example::

   catkin_add_env_hooks(my_prefix SHELLS bash tcsh zsh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

 looks for files env-hooks/my_prefix.[bash|tcsh|zsh]((.(devel|install)space)?.[em|in])?

 :param file_prefix: the filename prefix
 :type file_prefix: string
 :param SHELLS: the shell extensions (e.g.: sh bat bash zsh tcsh)
 :type SHELLS: list of strings
 :param DIRECTORY: the directory (default: ${CMAKE_CURRENT_SOURCE_DIR})
 :type DIRECTORY: string
 :param SKIP_INSTALL: if specified the env hooks are only generated
   in the devel space but not installed
 :type SKIP_INSTALL: option



.. _`catkin_add_executable_with_gtest_ref`:

`catkin_add_executable_with_gtest`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_add_executable_with_gtest(target)

 *[function defined in test/gtest.cmake]*


 Add a GTest executable target.

 An executable target is created with the source files, it is linked
 against GTest.
 If you also want to register the executable as a test use
 ``catkin_add_gtest()`` instead.

 :param target: the target name
 :type target: string
 :param source_files: a list of source files used to build the test
   executable
 :type source_files: list of strings

 Additionally, the option EXCLUDE_FROM_ALL can be specified.


.. _`catkin_add_gtest_ref`:

`catkin_add_gtest`
~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_add_gtest(target)

 *[function defined in test/gtest.cmake]*


 Add a GTest based test target.

 An executable target is created with the source files, it is linked
 against GTest and added to the set of unit tests.

 .. note:: The test can be executed by calling the binary directly
   or using: ``make run_tests_${PROJECT_NAME}_gtest_${target}``

 :param target: the target name
 :type target: string
 :param source_files: a list of source files used to build the test
   executable
 :type source_files: list of strings
 :param TIMEOUT: currently not supported
 :type TIMEOUT: integer
 :param WORKING_DIRECTORY: the working directory when executing the
   executable
 :type WORKING_DIRECTORY: string



.. _`catkin_add_nosetests_ref`:

`catkin_add_nosetests`
~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_add_nosetests(path)

 *[function defined in test/nosetests.cmake]*


 Add Python nose tests.

 Nose collects tests from the directory ``dir`` automatically.

 .. note:: The test can be executed by calling ``nosetests``
   directly or using:
   `` make run_tests_${PROJECT_NAME}_nosetests_${dir}``
   (where slashes in the ``dir`` are replaced with periods)

 :param path: a relative or absolute directory to search for
   nosetests in or a relative or absolute file containing tests
 :type path: string
 :param DEPENDENCIES: the targets which must be built before executing
   the test
 :type DEPENDENCIES: list of strings
 :param TIMEOUT: the timeout for individual tests in seconds
   (default: 60)
 :type TIMEOUT: integer
 :param WORKING_DIRECTORY: the working directory when executing the
   tests (this option can only be used when the ``path`` argument is a
   file  but not when it is a directory)
 :type WORKING_DIRECTORY: string



.. _`catkin_download_ref`:

`catkin_download`
~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_download(target, url)

 *[function defined in catkin_download.cmake]*


 Download a file containing data from a URL.

 It is commonly used to download larger data files which should not be
 stored in the repository.

 .. note:: It is not recommended to rely on downloaded data during
   a configure / make cycle since this prevents building the package
   when no network connectivity is available.

 .. note:: The target will be registered as a dependency
   of the "download_extra_data" target.

 :param target: the target name
 :type target: string
 :param url: the url to download
 :type url: string

 :param DESTINATION: the directory where the file is downloaded to
   (default: ${PROJECT_BINARY_DIR})
 :type DESTINATION: string
 :param FILENAME: the filename of the downloaded file
   (default: the basename of the url)
 :type FILENAME: string
 :param MD5: the expected md5 hash to compare against
   (default: empty, skipping the check)
 :type MD5: string

 Additionally, options EXCLUDE_FROM_ALL and REQUIRED can be specified.


.. _`catkin_download_test_data_ref`:

`catkin_download_test_data`
~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_download_test_data(target, url)

 *[function defined in test/catkin_download_test_data.cmake]*

 :param DESTINATION: the directory where the file is downloaded to
   (default: ${PROJECT_BINARY_DIR})
 :type DESTINATION: string
 :param FILENAME: the filename of the downloaded file
   (default: the basename of the url)
 :type FILENAME: string
 :param MD5: the expected md5 hash to compare against
   (default: empty, skipping the check)
 :type MD5: string

 Additionally, option REQUIRED can be specified.


.. _`catkin_filter_libraries_for_build_configuration_ref`:

`catkin_filter_libraries_for_build_configuration`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_filter_libraries_for_build_configuration(VAR)

 *[macro defined in catkin_libraries.cmake]*


 Filter libraries based on optional build configuration keywords.

 :param VAR: the output variable name
 :type VAR: string
 :param ARGN: a list of libraries
 :type ARGN: list of strings
 :param BUILD_TYPE: a keyword for the build type (default:
   ``CMAKE_BUILD_TYPE``)
 :type BUILD_TYPE: list of strings



.. _`catkin_install_python_ref`:

`catkin_install_python`
~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_install_python(signature)

 *[function defined in catkin_install_python.cmake]*


 Install Python files and update their shebang lines
 to use a different Python executable.

 The signature:

   catkin_install_python(PROGRAMS files... DESTINATION <dir> [OPTIONAL])

 See the documentation for CMake install() function for more information.



.. _`catkin_metapackage_ref`:

`catkin_metapackage`
~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_metapackage()

 *[function defined in catkin_metapackage.cmake]*


 It installs the package.xml file of a metapackage.

 .. note:: It must be called once for each metapackage.  Best
   practice is to call this macro early in your root CMakeLists.txt,
   immediately after calling ``project()`` and
   ``find_package(catkin REQUIRED)``.

 :param DIRECTORY: the path to the package.xml file if not in the same
   location as the CMakeLists.txt file
 :type DIRECTORY: string



.. _`catkin_pack_libraries_with_build_configuration_ref`:

`catkin_pack_libraries_with_build_configuration`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_pack_libraries_with_build_configuration(VAR)

 *[macro defined in catkin_libraries.cmake]*


 Pack a list of libraries with optional build configuration keywords.
 Each keyword is joined with its library using a separator.
 A packed library list can be deduplicated correctly.

 :param VAR: the output variable name
 :type VAR: string
 :param ARGN: a list of libraries
 :type ARGN: list of strings



.. _`catkin_package_ref`:

`catkin_package`
~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_package()

 *[macro defined in catkin_package.cmake]*


 It installs the package.xml file, and it generates code for
 ``find_package`` and ``pkg-config`` so that other packages can get
 information about this package.  For this purpose the information
 about include directories, libraries, further dependencies and
 CMake variables are used.

 .. note:: It must be called once for each package.  It is indirectly
   calling``catkin_destinations()`` which will provide additional
   output variables.  Please make sure to call ``catkin_package()``
   before using those variables.

 :param INCLUDE_DIRS: ``CMAKE_CURRENT_SOURCE_DIR``-relative paths to
   C/C++ includes
 :type INCLUDE_DIRS: list of strings
 :param LIBRARIES: names of library targets that will appear in the
   ``catkin_LIBRARIES`` and ``${PROJECT_NAME}_LIBRARIES`` of other
   projects that search for you via ``find_package``.  Currently
   this will break if the logical target names are not the same as
   the installed names.
 :type LIBRARIES: list of strings
 :param CATKIN_DEPENDS: a list of catkin projects which this project
   depends on.  It is used when client code finds this project via
   ``find_package()`` or ``pkg-config``.  Each project listed will in
   turn be ``find_package``\ -ed or is states as ``Requires`` in the
   .pc file.  Therefore their ``INCLUDE_DIRS`` and ``LIBRARIES`` will
   be appended to ours.  Only catkin projects should be used where it
   be guarantee that they are *find_packagable* and have pkg-config
   files.
 :type CATKIN_DEPENDS: list of strings
 :param DEPENDS: a list of CMake projects which this project depends
   on.  Since they might not be *find_packagable* or lack a pkg-config
   file their ``INCLUDE_DIRS`` and ``LIBRARIES`` are passed directly.
   This requires that it has been ``find_package``\ -ed before.
 :type DEPENDS: list of strings
 :param CFG_EXTRAS: a CMake file containing extra stuff that should
   be accessible to users of this package after
   ``find_package``\ -ing it.  This file must live in the
   subdirectory ``cmake`` or be an absolute path.
   All passed extra files must have unique basenames since they are
   being installed into a single folder.
   Various additional file extension are possible:
   for a plain cmake file just ``.cmake``, for files expanded using
   CMake's ``configure_file()`` use ``.cmake.in`` or for files expanded
   by empy use ``.cmake.em``.  The templates can distinguish between
   devel- and installspace using the boolean variables ``DEVELSPACE``
   and ``INSTALLSPACE``.  For templated files it is also possible to
   use the extensions ``.cmake.develspace.(in|em)`` or
   ``.cmake.installspace.(em|in)`` to generate the files only for a
   specific case.
   If the global variable ${PROJECT_NAME}_CFG_EXTRAS is set it will be
   prepended to the explicitly passed argument.
 :type CFG_EXTRAS: string
 :param EXPORTED_TARGETS: a list of target names which usually generate
   code. Downstream packages can depend on these targets to ensure that
   code is generated before it is being used. The generated CMake config
   file will ensure that the targets exists.
   If the global variable ${PROJECT_NAME}_EXPORTED_TARGETS is
   set it will be prepended to the explicitly passed argument.
 :type EXPORTED_TARGETS: list of strings
 :param SKIP_CMAKE_CONFIG_GENERATION: the option to skip the generation
   of the CMake config files for the package
 :type SKIP_CMAKE_CONFIG_GENERATION: bool
 :param SKIP_PKG_CONFIG_GENERATION: the option to skip the generation of
   the pkg-config file for the package
 :type SKIP_PKG_CONFIG_GENERATION: bool

 Example:
 ::

   catkin_package(
     INCLUDE_DIRS include
     LIBRARIES projlib1 projlib2
     CATKIN_DEPENDS roscpp
     DEPENDS Eigen
     CFG_EXTRAS proj-extras[.cmake|.cmake.in|.cmake(.develspace|.installspace)?.em]
   )



.. _`catkin_package_xml_ref`:

`catkin_package_xml`
~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_package_xml()

 *[macro defined in catkin_package_xml.cmake]*


 Parse package.xml from ``CMAKE_CURRENT_SOURCE_DIR`` and
 make several information available to CMake.

 .. note:: It is called automatically by ``catkin_package()`` if not
   called manually before.  It must be called once in each package,
   after calling ``project()`` where the project name must match the
   package name.  The macro should only be called manually if the
   variables are use to parameterize ``catkin_package()``.

 :param DIRECTORY: the directory of the package.xml (default
   ``${CMAKE_CURRENT_SOURCE_DIR}``).
 :type DIRECTORY: string

 :outvar <packagename>_VERSION: the version number
 :outvar <packagename>_MAINTAINER: the name and email of the
   maintainer(s)
 :outvar _CATKIN_CURRENT_PACKAGE: the name of the package from the
   manifest

 .. note:: It is calling ``catkin_destinations()`` which will provide
   additional output variables.



.. _`catkin_python_setup_ref`:

`catkin_python_setup`
~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_python_setup()

 *[function defined in catkin_python_setup.cmake]*

 This macro will interrogate the Python setup.py file in
 ``${${PROJECT_NAME}_SOURCE_DIR}``, and then creates forwarding
 Python :term:`pkgutil` infrastructure in devel space
 accordingly for the scripts and packages declared in setup.py.

 Doing so enables mixing :term:`generated code` in
 devel space with :term:`static code` from sourcespace within a
 single Python package.

 In addition, it adds the install command of
 distutils/setuputils to the install target.

 .. note:: If the project also uses genmsg message generation via
   ``generate_messages()`` this function must be called before.



.. _`catkin_replace_imported_library_targets_ref`:

`catkin_replace_imported_library_targets`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_replace_imported_library_targets(VAR)

 *[macro defined in catkin_libraries.cmake]*


 Replace imported library target names with the library name.

 :param VAR: the output variable name
 :type VAR: string
 :param ARGN: a list of libraries
 :type ARGN: list of strings



.. _`catkin_unpack_libraries_with_build_configuration_ref`:

`catkin_unpack_libraries_with_build_configuration`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_unpack_libraries_with_build_configuration(VAR)

 *[macro defined in catkin_libraries.cmake]*


 Unpack a list of libraries with optional build configuration keyword prefixes.
 Libraries prefixed with a keyword are split into the keyword and the library.

 :param VAR: the output variable name
 :type VAR: string
 :param ARGN: a list of libraries
 :type ARGN: list of strings



Non-public CMake functions / macros
-----------------------------------

 * :cmake:macro:`_generate_function_if_testing_is_disabled`
 * :cmake:macro:`_set_cmake_policy_to_new_if_available`
 * :cmake:macro:`_warn_if_skip_testing`
 * :cmake:macro:`catkin_destinations`
 * :cmake:macro:`catkin_run_tests_target`
 * :cmake:macro:`catkin_workspace`
 * :cmake:macro:`list_append_deduplicate`
 * :cmake:macro:`list_append_unique`
 * :cmake:macro:`stamp`
 * :cmake:macro:`string_starts_with`

.. _`_generate_function_if_testing_is_disabled_ref`:

`_generate_function_if_testing_is_disabled`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: _generate_function_if_testing_is_disabled(funcname)

 *[macro defined in test/tests.cmake]*

 creates a dummy function in case testing has been explicitly disabled (and not only skipping)
 which outputs an error message when being invoked

.. _`_set_cmake_policy_to_new_if_available_ref`:

`_set_cmake_policy_to_new_if_available`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: _set_cmake_policy_to_new_if_available(policy)

 *[macro defined in all.cmake]*

 enable all new policies (if available)

.. _`_warn_if_skip_testing_ref`:

`_warn_if_skip_testing`
~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: _warn_if_skip_testing(funcname)

 *[macro defined in test/tests.cmake]*

 checks if a function has been called while testing is skipped
 and outputs a warning message

.. _`catkin_destinations_ref`:

`catkin_destinations`
~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_destinations()

 *[macro defined in catkin_destinations.cmake]*


 Set several path suffixes for install destinations.

 :outvar CATKIN_PACKAGE_BIN_DESTINATION:
   See :cmake:data:`CATKIN_PACKAGE_BIN_DESTINATION`.
 :outvar CATKIN_PACKAGE_ETC_DESTINATION:
   See :cmake:data:`CATKIN_PACKAGE_ETC_DESTINATION`.
 :outvar CATKIN_PACKAGE_INCLUDE_DESTINATION:
   See :cmake:data:`CATKIN_PACKAGE_INCLUDE_DESTINATION`.
 :outvar CATKIN_PACKAGE_LIB_DESTINATION:
   See :cmake:data:`CATKIN_PACKAGE_LIB_DESTINATION`.
 :outvar CATKIN_PACKAGE_PYTHON_DESTINATION:
   See :cmake:data:`CATKIN_PACKAGE_PYTHON_DESTINATION`.
 :outvar CATKIN_PACKAGE_SHARE_DESTINATION:
   See :cmake:data:`CATKIN_PACKAGE_SHARE_DESTINATION`.

 :outvar CATKIN_GLOBAL_BIN_DESTINATION:
   See :cmake:data:`CATKIN_GLOBAL_BIN_DESTINATION`.
 :outvar CATKIN_GLOBAL_ETC_DESTINATION:
   See :cmake:data:`CATKIN_GLOBAL_ETC_DESTINATION`.
 :outvar CATKIN_GLOBAL_INCLUDE_DESTINATION:
   See :cmake:data:`CATKIN_GLOBAL_INCLUDE_DESTINATION`.
 :outvar CATKIN_GLOBAL_LIB_DESTINATION:
   See :cmake:data:`CATKIN_GLOBAL_LIB_DESTINATION`.
 :outvar CATKIN_GLOBAL_LIBEXEC_DESTINATION:
   See :cmake:data:`CATKIN_GLOBAL_LIBEXEC_DESTINATION`.
 :outvar CATKIN_GLOBAL_PYTHON_DESTINATION:
   See :cmake:data:`CATKIN_GLOBAL_PYTHON_DESTINATION`.
 :outvar CATKIN_GLOBAL_SHARE_DESTINATION:
   See :cmake:data:`CATKIN_GLOBAL_SHARE_DESTINATION`.


.. _`catkin_run_tests_target_ref`:

`catkin_run_tests_target`
~~~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_run_tests_target(type, name, xunit_filename)

 *[function defined in test/tests.cmake]*


 Create a test target, integrate it with the run_tests infrastructure
 and post-process the junit result.

 All test results go under ${CATKIN_TEST_RESULTS_DIR}/${PROJECT_NAME}

 This function is only used internally by the various
 catkin_add_*test() functions.


.. _`catkin_workspace_ref`:

`catkin_workspace`
~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_workspace()

 *[function defined in catkin_workspace.cmake]*


 Search all subfolders in the workspace for ``package.xml`` files.
 Based on the dependencies specified in the ``build_depends``,
 ``buildtool_depends`` and (as of package format version 2)
 ``test_depends`` tags it performs a topological sort and calls
 ``add_subdirectory()`` for each directory.

 The functions is only called in catkin's ``toplevel.cmake``, which
 is usually symlinked to the workspace root directory (which
 contains multiple packages).


.. _`list_append_deduplicate_ref`:

`list_append_deduplicate`
~~~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: list_append_deduplicate(listname)

 *[macro defined in list_append_deduplicate.cmake]*


 Append elements to a list and remove existing duplicates from the list.

 .. note:: Using CMake's ``list(APPEND ..)`` and
   ``list(REMOVE_DUPLICATES ..)`` is not sufficient since its
   implementation uses a set internally which makes the operation
   unstable.


.. _`list_append_unique_ref`:

`list_append_unique`
~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: list_append_unique(listname)

 *[macro defined in list_append_unique.cmake]*


 Append elements to a list if they are not already in the list.

 .. note:: Using CMake's ``list(APPEND ..)`` and
   ``list(REMOVE_DUPLICATES ..)`` is not sufficient since its
   implementation uses a set internally which makes the operation
   unstable.


.. _`stamp_ref`:

`stamp`
~~~~~~~

.. cmake:macro:: stamp(path)

 *[function defined in stamp.cmake]*


   :param path:  file name

   Uses ``configure_file`` to generate a file ``filepath.stamp`` hidden
   somewhere in the build tree.  This will cause cmake to rebuild its
   cache when ``filepath`` is modified.


.. _`string_starts_with_ref`:

`string_starts_with`
~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: string_starts_with(str, prefix, var)

 *[function defined in string_starts_with.cmake]*


 Check if a string starts with a prefix.

 :param str: the string
 :type str: string
 :param prefix: the prefix
 :type prefix: string
 :param var: the output variable name
 :type var: bool


Not documented CMake functions / macros
---------------------------------------

.. _`_catkin_package_ref`:

`_catkin_package`
~~~~~~~~~~~~~~~~~

.. cmake:macro:: _catkin_package()

 *[function defined in catkin_package.cmake]*

.. _`_catkin_package_xml_ref`:

`_catkin_package_xml`
~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: _catkin_package_xml(dest_dir)

 *[macro defined in catkin_package_xml.cmake]*

.. _`_strip_path_prefix_ref`:

`_strip_path_prefix`
~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: _strip_path_prefix(var, value, prefix)

 *[macro defined in test/nosetests.cmake]*

.. _`assert_ref`:

`assert`
~~~~~~~~

.. cmake:macro:: assert(VAR)

 *[function defined in assert.cmake]*

.. _`assert_file_exists_ref`:

`assert_file_exists`
~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: assert_file_exists(FILENAME, MESSAGE)

 *[function defined in assert.cmake]*

.. _`assert_unset_ref`:

`assert_unset`
~~~~~~~~~~~~~~

.. cmake:macro:: assert_unset(VAR)

 *[function defined in assert.cmake]*

.. _`atomic_configure_file_ref`:

`atomic_configure_file`
~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: atomic_configure_file(input, output)

 *[function defined in atomic_configure_file.cmake]*

.. _`catkin_doxygen_ref`:

`catkin_doxygen`
~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_doxygen(TARGET_NAME, SEARCH_DIRS)

 *[macro defined in tools/doxygen.cmake]*

.. _`catkin_generate_environment_ref`:

`catkin_generate_environment`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: catkin_generate_environment()

 *[function defined in catkin_generate_environment.cmake]*

.. _`catkin_stack_ref`:

`catkin_stack`
~~~~~~~~~~~~~~

.. cmake:macro:: catkin_stack()

 *[function defined in legacy.cmake]*

.. _`configure_shared_library_build_settings_ref`:

`configure_shared_library_build_settings`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: configure_shared_library_build_settings()

 *[function defined in tools/libraries.cmake]*

.. _`debug_message_ref`:

`debug_message`
~~~~~~~~~~~~~~~

.. cmake:macro:: debug_message(level)

 *[macro defined in debug_message.cmake]*

.. _`em_expand_ref`:

`em_expand`
~~~~~~~~~~~

.. cmake:macro:: em_expand(context_in, context_out, em_file_in, file_out)

 *[macro defined in em_expand.cmake]*

.. _`find_program_required_ref`:

`find_program_required`
~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: find_program_required(ARG_VAR, ARG_PROGRAM_NAME)

 *[function defined in find_program_required.cmake]*

.. _`find_python_module_ref`:

`find_python_module`
~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: find_python_module(module)

 *[function defined in empy.cmake]*

.. _`list_insert_in_workspace_order_ref`:

`list_insert_in_workspace_order`
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: list_insert_in_workspace_order(listname)

 *[macro defined in list_insert_in_workspace_order.cmake]*

.. _`safe_execute_process_ref`:

`safe_execute_process`
~~~~~~~~~~~~~~~~~~~~~~

.. cmake:macro:: safe_execute_process(cmd_keyword, arg1)

 *[macro defined in safe_execute_process.cmake]*

.. _`shell_ref`:

`shell`
~~~~~~~

.. cmake:macro:: shell(arg1)

 *[function defined in shell.cmake]*
