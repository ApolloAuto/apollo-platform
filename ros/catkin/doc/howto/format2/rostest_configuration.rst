.. _rostest_configuration_2:

Configuring rostest
-------------------

Use rostest_ whenever your unit tests require a roscore_ for running
ROS nodes.


package.xml
:::::::::::

Declare ``rostest`` as a test dependency, along with any other
test-only dependencies::

  <test_depend>rostest</test_depend>


CMakeLists.txt
::::::::::::::

You need a ``find_package()`` for ``rostest`` to define the necessary
CMake commands.  It is better *not* to use a second
``find_package(catkin ...)`` for test dependencies like rostest,
because that would reset important catkin CMake variables, making it
hard to build test programs.

Place both the ``find_package()`` and your test declarations inside
the conditional testing block::

  if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(tests/your_first_rostest.test)
    add_rostest(tests/your_second_rostest.test)
  endif()

In case you have a test that accepts arguments, you can pass them like
this::

  add_rostest(tests/your_rostest.test ARGS arg1:=true arg2:=false)

If your rostest needs extra data in order to run, you can use the
``catkin_download_test_data()`` to download the data.
Read more about :ref:`downloading_test_data_2`.
Then you can add a dependency between the rostest target and the
target from ``catkin_download_test_data()``, in order to download the
data before the rostest runs::

  if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    catkin_download_test_data(
      ${PROJECT_NAME}_32e.pcap
      http://download.ros.org/data/velodyne/32e.pcap
      MD5 e41d02aac34f0967c03a5597e1d554a9)

    add_rostest(tests/your_rostest.test DEPENDENCIES ${PROJECT_NAME}_32e.pcap)
  endif()

If your rostest also uses a gtest_ executable, there is a convenience
function::

  if (CATKIN_ENABLE_TESTING)
    add_rostest_gtest(your_gtest_node
                      tests/your_third_rostest.test
                      tests/your_gtest_node.cpp)
    target_link_libraries(your_gtest_node ${catkin_LIBRARIES})
  endif()

Any additional library dependencies would be added to the
``target_link_libraries()``, as usual.

For more details on writing and running rostests, see the rostest_
documentation.

.. _gtest: http://wiki.ros.org/gtest
.. _roscore: http://wiki.ros.org/roscore
.. _rostest: http://wiki.ros.org/rostest
