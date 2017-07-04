.. _migrating_from_format1_to_format2:

Migrating from format 1 to format 2
===================================

The differences between format 1 and format 2 only affect the
``package.xml`` with its dependencies.  REP-0140_ defines these
differences and provides their rationale.

``<package>``
:::::::::::::

The ``<package>`` tag determines which format to use, change it like
this::

  <package format="2">

``<depend>``
::::::::::::

This is a new tag, intended to reduce unnecessary repetition.  If your
format 1 package contained::

  <build_depend>foo</build_depend>
  <run_depend>foo</run_depend>

It should be replaced with::

  <depend>foo</depend>

In format 2, that is equivalent to::

  <build_depend>foo</build_depend>
  <build_export_depend>foo</build_export_depend>
  <exec_depend>foo</exec_depend>

``<run_depend>``
::::::::::::::::

This tag is no longer allowed.  Wherever found, it must be replaced::

  <run_depend>foo</run_depend>

In format 2, that is equivalent to these two new tags::

  <build_export_depend>foo</build_export_depend>
  <exec_depend>foo</exec_depend>

If the dependency is only used at run-time, only the ``<exec_depend>``
is needed.  If it is only exported to satisfy other packages' build
dependencies, use ``<build_export_depend>``.  If both are needed, this
may be a better choice::

  <depend>foo</depend>

``<test_depend>``
:::::::::::::::::

In format 2, this tag can satisfy build dependencies, not just those
needed for executing your tests.  Unlike format 1, ``<test_depend>``
may now refer to a package also declared as some other type of
dependency.

Some test-only dependencies that formerly required a
``<build_depend>``, should now be expressed using ``<test_depend>``.
For example::

  <build_depend>rostest</build_depend>

becomes::

  <test_depend>rostest</test_depend>

In your ``CMakeLists.txt`` make sure your test dependencies are only
referenced within the conditional test block::

  if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest(tests/your_first_rostest.test)
    add_rostest(tests/your_second_rostest.test)
  endif()

``<doc_depend>``
::::::::::::::::

This tag defines dependencies needed for building your documentation::

  <doc_depend>doxygen</doc_depend>
  <doc_depend>epydoc</doc_depend>
  <doc_depend>python-sphinx</doc_depend>
  <doc_depend>rosdoc_lite</doc_depend>

Those examples are automatically provided by the ROS build farm, but
there is no harm in declaring which you actually use.  They do not
create binary package dependencies, unless they were also declared
using some other dependency tag.


.. _REP-0140: http://ros.org/reps/rep-0140.html
