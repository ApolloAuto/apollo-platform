.. _run_tests_2:

Running unit tests
------------------

All unit tests can be run by invoking the ``run_tests`` target::

  catkin_make run_tests

It will make sure that the ``tests`` target which builds all unit
tests is invoked before.


Individual test targets
:::::::::::::::::::::::

Besides the global ``run_tests`` target each individual `gtest` /
`nosetest` / `rostest` can be triggered via its own target, e.g.::

  catkin_make run_tests_packagename_gtest_mytest

The list of available test targets can be explored by using the
target completion of catkin_make::

  catkin_make run_tests<TAB><TAB>

Using these target it is also possible to run all unit tests of a
package or all tests from a package of a specific type::

  catkin_make run_tests_packagename
  catkin_make run_tests_packagename_gtest


Summary of unit test results
::::::::::::::::::::::::::::

After running unit tests the results are by default placed into the
``test_results`` folder which is located inside the build folder.
Each unit test will generate a JUnit XML result file.

To get a summary of the unit test results as well as references to
which tests contain errors or failed invoke::

  catkin_test_results build/test_results
