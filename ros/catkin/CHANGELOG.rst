^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package catkin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.19 (2017-03-06)
-------------------
* update --pkg help for catkin_make_isolated (`#853 <https://github.com/ros/catkin/issues/853>`_)
* add skipped / disabled tests to catkin_test_results summary (`#848 <https://github.com/ros/catkin/issues/848>`_)
* fix rollback logic for more than one value per environment variable and workspace (`#819 <https://github.com/ros/catkin/issues/819>`_)
* fix quoting of paths to handle spaces (`#808 <https://github.com/ros/catkin/issues/808>`_)
* improve doc about catkin_package(CFG_EXTRAS) (`#805 <https://github.com/ros/catkin/issues/805>`_)
* doc: fix format 2 howto to suggest to declare a build export dependency on "message_runtime"
* update documentation (`#847 <https://github.com/ros/catkin/pull/847>`_ `#849 <https://github.com/ros/catkin/pull/849>`_ `#854 <https://github.com/ros/catkin/pull/854>`_)

0.6.18 (2016-03-18)
-------------------
* expose format 2 style dependencies as CMake variables (`#787 <https://github.com/ros/catkin/issues/787>`_)

0.6.17 (2016-03-03)
-------------------
* fix docs: nosetest target names use periods (`#781 <https://github.com/ros/catkin/issues/781>`_)
* add custom message explaining CMake find_package error messages (`#780 <https://github.com/ros/catkin/issues/780>`_)
* fix regression with DESTDIR introduced in 0.6.16 (`#755 <https://github.com/ros/catkin/issues/755>`_)
* avoid adding nonexistent paths to environment variables (`#777 <https://github.com/ros/catkin/issues/777>`_)
* ensure that Python install destination exists (`#775 <https://github.com/ros/catkin/issues/775>`_, https://github.com/ros/catkin/issues/776)
* set commonly predefines attributes when interrogating setup.py files (`#770 <https://github.com/ros/catkin/issues/770>`_)
* align Python script directory recommendations with REP-0008 (`#769 <https://github.com/ros/catkin/issues/769>`_)
* fix default value for _workspaces in find_in_workspaces (`#768 <https://github.com/ros/catkin/issues/768>`_)
* improve robustness of exec call interogating setup.py files (`#766 <https://github.com/ros/catkin/issues/766>`)
* fix reinstalling Python files installed by catkin_install_python after modifying them (`#764 <https://github.com/ros/catkin/issues/764>`_)
* fix project specific clean_test_results targets (`#762 <https://github.com/ros/catkin/issues/762>`_)
* update generated CMake API

0.6.16 (2015-11-09)
-------------------
* remove -x in Python distutils shell script (`#755 <https://github.com/ros/catkin/issues/755>`_)

0.6.15 (2015-09-19)
-------------------
* adding check to prevent tool cross talk (`#743 <https://github.com/ros/catkin/issues/743>`_)
* only clean the test result ending with .xml of the current project (`#737 <https://github.com/ros/catkin/issues/737>`_)
* fix regression in find_in_workspaces (`#739 <https://github.com/ros/catkin/issues/739>`_)
* fix setup.py package_dir location logic (`#751 <https://github.com/ros/catkin/issues/751>`_)

0.6.14 (2015-04-20)
-------------------
* support zsh with NOCLOBBER enabled (`#734 <https://github.com/ros/catkin/pull/734>`_)

0.6.13 (2015-04-17)
-------------------
* allow setting CATKIN_GLOBAL_LIBEXEC_DESTINATION to libexec (`#713 <https://github.com/ros/catkin/pull/713>`_)

0.6.12 (2015-04-16)
-------------------
* remove CATKIN_TEST_RESULTS_DIR environment variable (`#728 <https://github.com/ros/catkin/issues/728>`_)
* catkin_test_results will output skipped xml files only when --all is being passed (`#733 <https://github.com/ros/catkin/pull/733>`_)
* extract catkin_add_executable_with_gtest() from catkin_add_gtest() (`#726 <https://github.com/ros/catkin/issues/726>`_)
* separate download function from tests (`#633 <https://github.com/ros/catkin/issues/633>`_)
* only install environment hooks for catkin_make(_isolated) completion in the catkin package (`#732 <https://github.com/ros/catkin/issues/732>`_)
* avoid warning with CMake 3.1 and newer (`#731 <https://github.com/ros/catkin/issues/731>`_)
* quote command in "Reproduce this error" instructions (`#730 <https://github.com/ros/catkin/issues/730>`_)
* fix Python error when working with non-ascii characters in catkin workspace path (`#724 <https://github.com/ros/catkin/issues/724>`_)
* use $TMPDIR for temporary _setup_util.py file if set (`#710 <https://github.com/ros/catkin/issues/710>`_)
* fix regex for library config types (`#723 <https://github.com/ros/catkin/issues/723>`_)
* fix potential race condition in download_checkmd5.py (`#715 <https://github.com/ros/catkin/issues/715>`_)
* output package whitelist / blacklist if set (`#714 <https://github.com/ros/catkin/issues/714>`_)
* add --verbose option to catkin_test_results to show the content of result files (`#705 <https://github.com/ros/catkin/issues/705>`_)
* source in reset zsh emulation mode  (`#686 <https://github.com/ros/catkin/issues/686>`_)
* improve help text for --only-pkg-with-deps (`#706 <https://github.com/ros/catkin/issues/706>`_)

0.6.11 (2014-12-29)
-------------------
* fix return code for tests run by ctest (`#703 <https://github.com/ros/catkin/issues/703>`_)

0.6.10 (2014-12-22)
-------------------
* check changes to -D args CATKIN_DEVEL_PREFIX / CMAKE_INSTALL_PREFIX when considering to reinvoke cmake (`#700 <https://github.com/ros/catkin/issues/700>`_)
* add --use-ninja option to catkin_make(_isolated) to use ninja instead of make (`#693 <https://github.com/ros/catkin/issues/693>`_)
* remove 'emulate sh' from setup.zsh (`#686 <https://github.com/ros/catkin/issues/686>`_)
* set terminal title only when isatty (`#687 <https://github.com/ros/catkin/issues/687>`_)
* add description to catkin_make for ignoring packages
* add suggestion to use catkin_make_isolated for non-homogeneous workspaces
* refactor code from run_tests.py into Python module (`#678 <https://github.com/ros/catkin/issues/678>`_)

0.6.9 (2014-07-11)
------------------
* fix regression from 0.6.8 (`#676 <https://github.com/ros/catkin/issues/676>`_)

0.6.8 (2014-07-10)
------------------
* make nosetests --xunit-file argument an absolute path to work around nose bug 779 (`#659 <https://github.com/ros/catkin/issues/659>`_)
* fix handling of CMake packages which do not install any files (`#665 <https://github.com/ros/catkin/issues/665>`_)
* fix gtest on Arch Linux and others (`#663 <https://github.com/ros/catkin/issues/663>`_)
* improve generation of .catkin marker file (`#671 <https://github.com/ros/catkin/issues/671>`_, `#676 <https://github.com/ros/catkin/issues/676>`_)
* escape messages to avoid CMake warning (`#667 <https://github.com/ros/catkin/issues/667>`_)
* fix CMake warning for doxygen target with CMake 3 (`#660 <https://github.com/ros/catkin/issues/660>`_)
* avoid using ARGN for efficiency (`#669 <https://github.com/ros/catkin/issues/669>`_)

0.6.7 (2014-06-16)
------------------
* reset CATKIN_SHELL in devel space generated env.sh files (`#652 <https://github.com/ros/catkin/issues/652>`_, `#655 <https://github.com/ros/catkin/issues/655>`_)
* ignore cd path echo when using CDPATH (`#654 <https://github.com/ros/catkin/issues/654>`_)
* use PYTHON_EXECUTABLE for _setup_util.py (`#646 <https://github.com/ros/catkin/issues/646>`_)
* expose PYTHON_EXECUTABLE to environment hook .em templates (`#645 <https://github.com/ros/catkin/issues/645>`_)
* catkin_prepare_release:

  * add --no-push to catkin_prepare_release (`#657 <https://github.com/ros/catkin/issues/657>`_)
  * flush stdin before asking for input (`#658 <https://github.com/ros/catkin/issues/658>`_)

0.6.6 (2014-06-02)
------------------
* fix rollback of environment when workspace has been deleted (`#641 <https://github.com/ros/catkin/issues/641>`_)
* fix argument handling when cm / cmi is invoked in a symlinked folder (`#638 <https://github.com/ros/catkin/issues/638>`_)

0.6.5 (2014-05-25)
------------------
* fix regression in catkin_make_isolated from 0.6.4 (`#624 <https://github.com//ros/catkin/issues/624>`_)
* fix problem when catkin_make / catkin_make_isolated is invoked in a symlinked folder (`#638 <https://github.com//ros/catkin/issues/638>`_)

0.6.4 (2014-05-07)
------------------
* add architecture independent flag in package.xml (`#625 <https://github.com/ros/catkin/issues/625>`_)
* add error message for circular dependencies in catkin_make_isolated and catkin_topological_order (`#607 <https://github.com/ros/catkin/issues/607>`_, `#608 <https://github.com/ros/catkin/issues/608>`_)
* add multiarch support for catkin environment files (`#545 <https://github.com/ros/catkin/issues/545>`_)
* add workspace marker file for catkin_make / catkin_make_isolated (`#304 <https://github.com/ros/catkin/issues/304>`_)
* allow better performance for repeated invocations of find_in_workspaces()
* consider test_depends for topolocial order (`#612 <https://github.com/ros/catkin/issues/612>`_)
* invoke empy with specific Python interpreter (`#620 <https://github.com/ros/catkin/issues/620>`_)
* support setting ${PROJECT_NAME}_LIBRARIES before invoking catkin_package() (`#609 <https://github.com/ros/catkin/issues/609>`_)
* update package manifest to format 2 (`#619 <https://github.com/ros/catkin/issues/619>`_)
* fixes:

  * fix catkin_find to not return path with '/.' suffix (`#621 <https://github.com/ros/catkin/issues/621>`_)
  * fix python path setting for plain cmake workspaces (`#618 <https://github.com/ros/catkin/issues/618>`_)
  * improve unicode handling (`#615 <https://github.com/ros/catkin/issues/615>`_)
  * replace CMake usage of IMPORTED_IMPLIB with IMPORTED_LOCATION (`#616 <https://github.com/ros/catkin/issues/616>`_)
  * do not call chpwd hooks in setup.zsh (`#613 <https://github.com/ros/catkin/issues/613>`_)
  * set catkin_* variables only when find_package(catkin COMPONENTS ...) (`#629 <https://github.com/ros/catkin/issues/629>`_)

0.6.3 (2014-03-04)
------------------
* allow passing absolute INCLUDE_DIRS via catkin_package() into CMake config file in install space (`#600 <https://github.com/ros/catkin/issues/600>`_, `#601 <https://github.com/ros/catkin/issues/601>`_)
* improve error messages for wrong include dirs

0.6.2 (2014-02-24)
------------------
* use underlay workspaces when calculating topological order (`#590 <https://github.com/ros/catkin/issues/590>`_)
* remove restriction to run unit test single threaded (`#597 <https://github.com/ros/catkin/issues/597>`_)
* support using nosetests with Python3 (`#593 <https://github.com/ros/catkin/issues/593>`_)
* remove invalid symbolic links of workspace level CMakeLists.txt file (`#591 <https://github.com/ros/catkin/issues/591>`_)
* remove debug_message usage from generated pkgConfig.cmake files (`#583 <https://github.com/ros/catkin/issues/583>`_)
* use catkin_install_python() to install Python scripts (`#596 <https://github.com/ros/catkin/issues/596>`_)
* fix unicode error with japanese LANG (`#578 <https://github.com/ros/catkin/issues/578>`_)
* fix gtest include dir when using gtest inside the workspace (`#585 <https://github.com/ros/catkin/issues/585>`_)

0.6.1 (2014-01-29)
------------------
* remove more obsolete files (`#582 <https://github.com/ros/catkin/issues/582>`_)

0.6.0 (2014-01-29)
------------------
* remove deprecated functionality (`#582 <https://github.com/ros/catkin/issues/582>`_)
* remove eigen CMake config files (`#492 <https://github.com/ros/catkin/issues/492>`_)
* fix EXPORTED_TARGETS argument for catkin_package() (`#581 <https://github.com/ros/catkin/issues/581>`_)

0.5.80 (2014-01-27)
-------------------
* fix library deduplication (`#565 <https://github.com/ros/catkin/issues/565>`_)

0.5.79 (2014-01-09)
-------------------
* fix handling of not found imported libraries (`#565 <https://github.com/ros/catkin/issues/565>`_)

0.5.78 (2014-01-07)
-------------------
* add support for py_modules in setup.py (`#399 <https://github.com/ros/catkin/issues/399>`_)
* fix order and deduplication of of catkin_LIBRARIES (`#558 <https://github.com/ros/catkin/issues/558>`_)
* update handling of library deduplication to support build configuration keywords in the whole process (`#565 <https://github.com/ros/catkin/issues/565>`_)
* Python 3 related:

  * python 3 compatibility (`#544 <https://github.com/ros/catkin/issues/544>`_)
  * add option to specify Python version (`#570 <https://github.com/ros/catkin/issues/570>`_)
  * add catkin_install_python function (`#573 <https://github.com/ros/catkin/issues/573>`_)

* catkin_make and catkin_make_isolated:

  * add --only-pkg-with-deps option
  * fix completion for -DCMAKE_BUILD_TYPE and -DCATKIN_ENABLE_TESTING

* catkin_make_isolated:

  * add empty line between packages for better readability
  * update command line doc (`#534 <https://github.com/ros/catkin/issues/534>`_)
  * fix generated setup.sh for plain cmake package when path contains spaces
  * fix to actually build run_depends before the things that depend on them
  * fix check if environment exists with DESTDIR

* update setup scripts to use builtin cd command (`#542 <https://github.com/ros/catkin/issues/542>`_)
* improve docs on system library dependencies (`#552 <https://github.com/ros/catkin/issues/552>`_)
* install scripts under bin via setup.py (`#555 <https://github.com/ros/catkin/issues/555>`_)
* update download script to resume downloads when server supports it (`#559 <https://github.com/ros/catkin/issues/559>`_)
* add error handling when _setup_util.py fails, e.g. due to disk full (`#561 <https://github.com/ros/catkin/issues/561>`_)
* add atomic_configure_file to work support parallel builds better (`#566 <https://github.com/ros/catkin/issues/566>`_)
* update catkin_test_results usage message to mention return code (`#576 <https://github.com/ros/catkin/issues/576>`_)
* prefix invocation of python script with PYTHON_EXECUTABLE (`ros/genpy#23 <https://github.com/ros/genpy/issues/23>`_)
* update toplevel.cmake to use catkin_find_pkg instead of trying a fixed location to find catkin in the workspace
* modify gtest message level to only warn if gtest is not available but a package trys to register a gtest
* update run_tests to work robustly in parallel
* use sys.exit and not just exit
* catkin_prepare_release: undo quoting of commit messages

0.5.77 (2013-10-09)
-------------------
* catkin_prepare_release: show tag name explicitly when using tag prefix (`#528 <https://github.com/ros/catkin/issues/528>`_)

* catkin_make_isolated:

  * separate devel and install build folders for plain cmake packages cleanly without polluting namespace (`#532 <https://github.com/ros/catkin/issues/532>`_)
  * fix build folder naming (regression from 0.5.75)
  * fix setup generation if last package in workspace is plain cmake (`#530 <https://github.com/ros/catkin/issues/530>`_)
  * fix creating missing folder

0.5.76 (2013-10-06)
-------------------
* fix regression in catkin_make from 0.5.75

0.5.75 (2013-10-04)
-------------------
* add --all option to catkin_test_results
* improve svn support in catkin_prepare_release
* keep build folder of plain CMake packages completely isolated between devel and install (`#374 <https://github.com/ros/catkin/issues/374>`_)
* fix handling of spaces in path (`#519 <https://github.com/ros/catkin/issues/519>`_)
* fix generated setup.bash|zsh for isolated devel space (`#520 <https://github.com/ros/catkin/issues/520>`_)
* fix env.sh for plain cmake packages to not define _CATKIN_SETUP_DIR (`#521 <https://github.com/ros/catkin/issues/521>`_)
* fix crash when DESTDIR is specified but install not asked for (`#526 <https://github.com/ros/catkin/issues/526>`_)
* update doc for downloading test data from download.ros.org
* update environment hook doc (`#516 <https://github.com/ros/catkin/issues/516>`_)

0.5.74 (2013-09-18)
-------------------
* support multiple package names for catkin_make --pkg (`#504 <https://github.com/ros/catkin/issues/504>`_)
* improve help message on catkin_make_isolated --from-pkg
* fix include path in generated .pc files to be absolute (fix `#506 <https://github.com/ros/catkin/issues/506>`_, regression of 0.5.72)
* fix handling DESTDIR in catkin_make_isolated for plain cmake packages (`#499 <https://github.com/ros/catkin/issues/499>`_)
* fix catkin_python_setup to consider actual package names inside setup.py when signaling that an __init__.py file is installed to genmsg/dynamic_reconfigure (`ros/genmsg#34 <https://github.com/ros/genmsg/issues/34>`_)
* fix unsetting temporary variables - especially CATKIN_SETUP_DIR - to not influence next environment (`#505 <https://github.com/ros/catkin/issues/505>`_)

0.5.73 (2013-08-23)
-------------------
* fix include dirs in generated CMake config file when a non-catkin package depends on a catkin package (regression of `#490 <https://github.com/ros/catkin/issues/490>`_)

0.5.72 (2013-08-21)
-------------------
* make catkin packages relocatable (`#490 <https://github.com/ros/catkin/issues/490>`_)

* catkin_prepare_release:

  * add tagging changelog sections (`#489 <https://github.com/ros/catkin/issues/489>`_)
  * add warning when trying to release non-catkin packages (`#478 <https://github.com/ros/catkin/issues/478>`_)
  * add --tag-prefix option (`#447 <https://github.com/ros/catkin/issues/447>`_)

* catkin_make_isolated:

  * support installation with DESTDIR (`#490 <https://github.com/ros/catkin/issues/490>`_)
  * handle missing install target gracefully
  * fix message with command to reproduce (`#496 <https://github.com/ros/catkin/issues/496>`_)

* fix completion for catkin_make(_isolated) for older bash completion (`#485 <https://github.com/ros/catkin/issues/485>`_)
* fix Python3.2 compatibility (`#487 <https://github.com/ros/catkin/issues/487>`_)
* update documentation (`#438 <https://github.com/ros/catkin/issues/438>`_, `#467 <https://github.com/ros/catkin/issues/467>`_, `#495 <https://github.com/ros/catkin/issues/495>`_, `#497 <https://github.com/ros/catkin/issues/497>`_)

0.5.71 (2013-07-20)
-------------------
* catkin_make_isolated: show progress in terminal title bar on Linux and Darwin (`#482 <https://github.com/ros/catkin/issues/482>`_)
* catkin_prepare_release: add warning if package names contain upper case characters (`#473 <https://github.com/ros/catkin/issues/473>`_)
* catkin_make: fix handling of non-ascii characters (`#476 <https://github.com/ros/catkin/issues/476>`_)
* fix dry tests by disabling CTest for dry packages (`#483 <https://github.com/ros/catkin/issues/483>`_)
* update documentation (`#474 <https://github.com/ros/catkin/issues/474>`_)

0.5.70 (2013-07-14)
-------------------
* add warning if versioned dependencies are not fulfilled (`#472 <https://github.com/ros/catkin/issues/472>`_)
* catkin_prepare_release:

  * add checks for local changes in the working copy (`#471 <https://github.com/ros/catkin/issues/471>`_)
  * remove '--push' option in favor of interactive questions
  * improve error message when branch is not tracked (`#463 <https://github.com/ros/catkin/issues/463>`_)
  * colorize output for improved readability

* catkin_make_isolated:

  * suppress traceback when a package fails to build (`#470 <https://github.com/ros/catkin/issues/470>`_)
  * improve output to include 'cd' command to reproduce a failing command more easily

* fix initialization issue with CATKIN_ENABLE_TESTING variable (`#464 <https://github.com/ros/catkin/issues/464>`_)
* find nosetest under different name on QNX (`#461 <https://github.com/ros/catkin/issues/461>`_)
* update documentation (`#438 <https://github.com/ros/catkin/issues/438>`_, `#465 <https://github.com/ros/catkin/issues/465>`_, `#468 <https://github.com/ros/catkin/issues/468>`_)

0.5.69 (2013-07-05)
-------------------
* disable CTest on the farm since it breaks the binarydeb build with the current Debian rules files (`#460 <https://github.com/ros/catkin/issues/460>`_)
* skip generating CATKIN_IGNORE marker file when build space equals source space (`#459 <https://github.com/ros/catkin/issues/459>`_)
* fix warning message if gtest is not found (`#458 <https://github.com/ros/catkin/issues/458>`_)

0.5.68 (2013-07-03)
-------------------
* add option CATKIN_ENABLE_TESTING to configure without tests
* add CTest support for all registered test types: gtest, nosetest, rostest
* add the --from-pkg option to catkin_make_isolated
* catkin_prepare_release:

  * fix if git repo has multiple remotes (`#450 <https://github.com/ros/catkin/issues/450>`_)
  * modify to not change the remote repo by default, add option --push for old behavior (`#451 <https://github.com/ros/catkin/issues/451>`_)

* add 'prefix' to generated pkg-config files (`#444 <https://github.com/ros/catkin/issues/444>`_)
* add dummy target to catkin_EXPORTED_TARGETS if empty (`#453 <https://github.com/ros/catkin/issues/453>`_)
* expose SETUPTOOLS_DEB_LAYOUT as option again (`#418 <https://github.com/ros/catkin/issues/418>`_)
* suppress stacktrace when topologic_order raises within generating CMake files (`#442 <https://github.com/ros/catkin/issues/442>`_)
* fixes:

  * update check in generated pkgConfig.cmake files to work independent of cmake policy CMD0012 (`#452 <https://github.com/ros/catkin/issues/452>`_)
  * fix generating pkg-config files with empty -I directives (fix `#445 <https://github.com/ros/catkin/issues/445>`_)

* update documentation, especially about testing
* for a complete list of changes see the `commit log for 0.5.68 <https://github.com/ros/catkin/compare/0.5.67...0.5.68>`_

0.5.67 (2013-06-18)
-------------------
* fix --build and --directory auto completion for catkin_make(_isolated) (`#325 <https://github.com/ros/catkin/issues/325>`_)
* fix catkin_make(_isolated) auto completion on older versions of bash on OS X (`#325 <https://github.com/ros/catkin/issues/325>`_)
* add how-to documentation
* for a complete list of changes see the `commit log for 0.5.67 <https://github.com/ros/catkin/compare/0.5.66...0.5.67>`_

0.5.66 (2013-06-06)
-------------------
* add new CMake function catkin_download_test_data, mark download_test_data as deprecated (`#426 <https://github.com/ros/catkin/issues/426>`_, `#431 <https://github.com/ros/catkin/issues/431>`_)
* catkin_make and catkin_make_isolated:

  * add bash autocompletion (`#325 <https://github.com/ros/catkin/issues/325>`_)
  * allow passing MAKEFLAGS (`#402 <https://github.com/ros/catkin/issues/402>`_)

* catkin_make_isolated:

  * allow building empty workspace (`#423 <https://github.com/ros/catkin/issues/423>`_, `#425 <https://github.com/ros/catkin/issues/425>`_)
  * add --catkin-make-args which enables calling 'catkin_make_isolated run_tests' (`#414 <https://github.com/ros/catkin/issues/414>`_)

* catkin_prepare_release:

  * execute the commands rather than printing them (`#417 <https://github.com/ros/catkin/issues/417>`_)
  * warn about missing changelog files
  * validate metapackages (`#404 <https://github.com/ros/catkin/issues/404>`_)

* auto detect former SETUPTOOLS_DEB_LAYOUT (`#418 <https://github.com/ros/catkin/issues/418>`_)
* fixes:

  * fix error using sys.stdout.encoding without checking existance
  * remove linker flag -lrt for Android (`#430 <https://github.com/ros/catkin/issues/430>`_)
  * fix resetting IFS shell variable when it was unset before (`#427 <https://github.com/ros/catkin/issues/427>`_)
  * ensure to only return unique source paths from a workspace marker file (`#424 <https://github.com/ros/catkin/issues/424>`_)
  * catkin_make_isolated:

    * fix chaining of plain cmake packages (`#422 <https://github.com/ros/catkin/issues/422>`_)
    * fix --install error for non-catkin packages (`#411 <https://github.com/ros/catkin/issues/411>`_)
    * only try to remove Makefile if it exists (`#420 <https://github.com/ros/catkin/issues/420>`_, regression from 0.5.64)
    * fix detection of python paths for plain cmake packages

* update documentation:

  * update API doc of several CMake functions and macros
  * add generated CMake API to appear in the wiki (`#384 <https://github.com/ros/catkin/issues/384>`_)

* add and install LICENSE file (`#398 <https://github.com/ros/catkin/issues/398>`_)
* for a complete list of changes see the `commit log for 0.5.66 <https://github.com/ros/catkin/compare/0.5.65...0.5.66>`_

0.5.65 (2013-03-21)
-------------------
* remove including workspace.cmake if it exists, use -C and/or -DCMAKE_USER_MAKE_RULES_OVERRIDE instead (`#377 <https://github.com/ros/catkin/issues/377>`_)
* change catkin_test_results return code based on failing tests (`#392 <https://github.com/ros/catkin/issues/392>`_)
* apply CATKIN_BUILD_BINARY_PACKAGE for catkin (`#395 <https://github.com/ros/catkin/issues/395>`_)
* modify extra file handling to make the generated code relocatable (`#369 <https://github.com/ros/catkin/issues/369>`_)
* various improvements and fixes for Windows (`#388 <https://github.com/ros/catkin/issues/388>`_, `#390 <https://github.com/ros/catkin/issues/390>`_, `#391 <https://github.com/ros/catkin/issues/391>`_, `#393 <https://github.com/ros/catkin/issues/393>`_, `ros-infrastructure/catkin_pkg#35 <https://github.com/ros-infrastructure/catkin_pkg/issues/35>`_)
* fixes:

  * fix --force-cmake for catkin_make_isolation (regression from 0.5.64)
  * fix catkin_package_version and catkin_prepare_release when no packages are found (`#387 <https://github.com/ros/catkin/issues/387>`_)
  * fix catkin_prepare_release bumping wrong part of the version (`#386 <https://github.com/ros/catkin/issues/386>`_)
  * handle dependencies that are imported libraries (`#378 <https://github.com/ros/catkin/issues/378>`_)

* for a complete list of changes see the `commit log for 0.5.65 <https://github.com/ros/catkin/compare/0.5.64...0.5.65>`_

0.5.64 (2013-03-08)
-------------------
* disable env.sh without args (`#370 <https://github.com/ros/catkin/issues/370>`_)
* add --bump to catkin_prepare_release
* add EXPORTED_TARGETS to catkin_package() for cross-package target dependencies (`#368 <https://github.com/ros/catkin/issues/368>`_)
* enable cfg extras with absolute path and devel/install space specific
* enable additional cfg extra files via project specific global variable
* allow overriding CATKIN_TEST_RESULTS_DIR via cmake argument (`#365 <https://github.com/ros/catkin/issues/365>`_)
* add options to skip generation of cmake config and pkg-config files (`#360 <https://github.com/ros/catkin/issues/360>`_)
* catkin_make and catkin_make_isolated:

  * add --(c)make-args options to pass arbitrary arguments (`#376 <https://github.com/ros/catkin/issues/376>`_)

* catkin_make:

  * enable to build individual package (`#348 <https://github.com/ros/catkin/issues/348>`_)
  * enable to build metapackages without CMakeLists.txt (`#349 <https://github.com/ros/catkin/issues/349>`_)
  * add colorization of several error messages

* catkin_make_isolated:

  * requires metapackages to have a CMakeLists.txt (`#349 <https://github.com/ros/catkin/issues/349>`_)
  * force cmake for plain cmake packages when --install is toggled (`#374 <https://github.com/ros/catkin/issues/374>`_)
  * switched default output from quiet to verbose (`#331 <https://github.com/ros/catkin/issues/331>`_)
  * print full stacktrace to better diagnose internal problems (`#373 <https://github.com/ros/catkin/issues/373>`_)

* various improvements and fixes for Windows (`#276 <https://github.com/ros/catkin/issues/276>`_, `#351 <https://github.com/ros/catkin/issues/351>`_, `#358 <https://github.com/ros/catkin/issues/358>`_, `#359 <https://github.com/ros/catkin/issues/359>`_)
* fixes:

  * catkin_make and catkin_make_isolated:

    * ignore install space and any subspaces when searching packages (`#361 <https://github.com/ros/catkin/issues/361>`_)

  * catkin_make_isolated:

    * ensure that install space exists before trying to create a env.sh file for cmake projects (`#340 <https://github.com/ros/catkin/issues/340>`_)
    * fix generated env.sh file if first package is plain cmake (`#340 <https://github.com/ros/catkin/issues/340>`_)
    * remove old Makefile when cmake fails

  * fix sourcing some environment hooks multiple times
  * fix handling spaces in folder names (`ros/catkin#375 <https://github.com/ros/catkin/issues/375>`_)
  * fix usage of ROS_PARALLEL_JOBS (`#335 <https://github.com/ros/catkin/issues/335>`_)
  * prefix all temp variable in setup.sh (i.e. to not collide in zsh) (`#338 <https://github.com/ros/catkin/issues/338>`_)
  * setup.sh: check that temp file was created successfully, call rm command instead of potential alias (`#343 <https://github.com/ros/catkin/issues/343>`_)

* update documentation:

  * update doc on CFG_EXTRAS (`#353 <https://github.com/ros/catkin/issues/353>`_)
  * general catkin docs (`#357 <https://github.com/ros/catkin/issues/357>`_)

* for a complete list of changes see the `commit log for 0.5.64 <https://github.com/ros/catkin/compare/0.5.63...0.5.64>`_

0.5.63 (2013-01-24)
-------------------
* add sanity check to catkin_make that no CMakeLists.txt exists in the base path (`#328 <https://github.com/ros/catkin/issues/328>`_)
* catkin_make and catkin_make_isolated:

  * add -lN besides -jN (`#326 <https://github.com/ros/catkin/issues/326>`_)
  * support inverted color scheme (`#323 <https://github.com/ros/catkin/issues/323>`_)

* catkin_make_isolated:

  * allow building individual packages (`#322 <https://github.com/ros/catkin/issues/322>`_)
  * add support for -D options (`#306 <https://github.com/ros/catkin/issues/306>`_)
  * generate setup.sh file (`#327 <https://github.com/ros/catkin/issues/327>`_)
  * print instructions how to reproduce errors in case a command returns non-zero return code

* fixes:

  * readd old _setup_util.py to not break environment of workspaces built with 0.5.58 or older (`#329 <https://github.com/ros/catkin/issues/329>`_)
  * fix sourcing environment hooks
  * improve several error messages in case of problems (`#318 <https://github.com/ros/catkin/issues/318>`_, `#320 <https://github.com/ros/catkin/issues/320>`_)

* for a complete list of changes see the `commit log for 0.5.63 <https://github.com/ros/catkin/compare/0.5.62...0.5.63>`_

0.5.62 (2013-01-17)
-------------------
* fixes:

  * add check to catkin_make to force cmake when cmake-specific arguments change
  * force cmake on workspace changes in catkin_make_isolated (`#315 <https://github.com/ros/catkin/issues/315>`_)
  * source environment hooks for all workspace in correct order (`#316 <https://github.com/ros/catkin/issues/316>`_)
  * fix PYTHON_PACKAGES_DIR and SETUPTOOLS_ARG_EXTRA to be up-to-date when passing -DSETUPTOOLS_DEB_LAYOUT=.. (`#314 <https://github.com/ros/catkin/issues/314>`_)

* for a complete list of changes see the `commit log for 0.5.62 <https://github.com/ros/catkin/compare/0.5.61...0.5.62>`_

0.5.61 (2013-01-16)
-------------------
* yet another workaround for pkg-config handling static libraries (`#300 <https://github.com/ros/catkin/issues/300>`_)
* for a complete list of changes see the `commit log for 0.5.61 <https://github.com/ros/catkin/compare/0.5.60...0.5.61>`_

0.5.60 (2013-01-15)
-------------------
* work around for pkg-config reordering libs with no -l prefix (`#300 <https://github.com/ros/catkin/issues/300>`_)
* fix colorizing of unicode text in catkin_make and catkin_make_isolated (`#310 <https://github.com/ros/catkin/issues/310>`_)
* for a complete list of changes see the `commit log for 0.5.60 <https://github.com/ros/catkin/compare/0.5.59...0.5.60>`_

0.5.59 (2013-01-13)
-------------------
* improve performance to source setup.sh for nested workspaces (`#289 <https://github.com/ros/catkin/issues/289>`_)
* remove obsolete BUILD variables and find_package(ROS)
* improve error message when command execution or colorization fails (`#298 <https://github.com/ros/catkin/issues/298>`_)
* catkin_make keeps track of previously build packages and forces cmake on changes (`#290 <https://github.com/ros/catkin/issues/290>`_)
* fix pkg-config for absolute libraries (`#300 <https://github.com/ros/catkin/issues/300>`_)
* fix catkin_toplogical_order (`#299 <https://github.com/ros/catkin/issues/299>`_)
* make plain cmake install to devel when --install not present (`#302 <https://github.com/ros/catkin/issues/302>`_)
* fix creating test_results folder before trying to use it when running tests
* for a complete list of changes see the `commit log for 0.5.59 <https://github.com/ros/catkin/compare/0.5.58...0.5.59>`_

0.5.58 (2012-12-21)
-------------------
* first public release for Groovy
