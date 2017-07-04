^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cmake_modules
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.3 (2014-12-23)
------------------
* Added FindPoco.cmake, which migrated from the ros/class_loader repository.
* Update to FindXenomai.cmake
  find_package_handle_standard_args generates all caps variables (XENOMAI_FOUND), while this script is expected to create Xenomai_FOUND.
  This changeset creates the appropriately cased variable, but does not unset the all-caps variant for backwards-compatibility reasons (I typically unset it on new modules).
* Contributors: Adolfo Rodriguez Tsouroukdissian, Esteve Fernandez, William Woodall

0.3.2 (2014-10-27)
------------------
* Added CMake module for finding the UUID libraries
* Changed prepend of CMAKE_MODULE_PATH to append behaviour in order to allow prepending of external CMake modules.
* Added CMake module for finding GSL
* Contributors: Esteve Fernandez, Peter Lehner, William Woodall, v01d

0.3.1 (2014-05-07)
------------------
* Export architecture_independent flag in package.xml
* Fix extended CMAKE_MODULE_PATH variable when path contains spaces
* Mention the sequencing reqirement with an example
* Contributors: Dirk Thomas, Paul Mathieu, Scott K Logan, Tully Foote, William Woodall, phuicy

0.3.0 (2014-02-22)
------------------
* Added Numpy CMake module
* Added Eigen CMake module
  closed `#10 <https://github.com/ros/cmake_modules/issues/10>`_
* Removed use of absolute paths in extra files
  fixed `#9 <https://github.com/ros/cmake_modules/issues/9>`_
* Contributors: Vincent Rabaud, William Woodall

0.2.1 (2014-01-24)
------------------
* Adding CMake module for finding Xenomai RT kernel patch build flags
* Contributors: Jonathan Bohren, William Woodall

0.2.0 (2013-12-04)
------------------
* Added FindTBB.cmake version r36 from the findtbb package
* TinyXML: Add more comprehensive header documentation.

0.1.0 (2013-07-24)
------------------
* Initial release, includes the FindTinyXML.cmake CMake module
