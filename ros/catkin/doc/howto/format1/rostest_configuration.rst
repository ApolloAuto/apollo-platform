.. _rostest_configuration:

Configuring rostest
-------------------

Rostest_ is needed whenever unit tests require a roscore_ for running
ROS nodes.


package.xml
:::::::::::


The ``rostest`` package is needed at build time, because it defines
the ``add_rostest()`` CMake command::

  <build_depend>rostest</build_depend>

Do not declare a ``<test_depend>``, because it will conflict with the
required ``<build_depend>``.


CMakeLists.txt
::::::::::::::

You must also include ``rostest`` in your ``find_package()``
components.  Since these dependencies are for tests only you should
find them in a second invocation inside a conditional block::

  if (CATKIN_ENABLE_TESTING)
    find_package(catkin REQUIRED COMPONENTS rostest ...)
  endif()

Finally, declare your rostest launch scripts::

  if (CATKIN_ENABLE_TESTING)
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

If your rostest should contain a gtest executable you can use the
following convenient function::

  if (CATKIN_ENABLE_TESTING)
    add_rostest_gtest(your_third_rostest_target tests/your_third_rostest.test src/test/your_third_rostest.cpp [more cpp files])
    target_link_libraries(your_third_rostest_target [libraries to depend on, e.g. ${catkin_LIBRARIES}])
  endif()

For more information how to write and run rostests please go to the
Rostest_ wiki page.

.. _roscore: http://www.ros.org/wiki/roscore
.. _Rostest: http://ros.org/wiki/rostest
