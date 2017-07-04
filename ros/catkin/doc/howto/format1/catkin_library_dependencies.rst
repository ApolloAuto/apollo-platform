.. _catkin_library_dependencies_1:

C++ catkin library dependencies
-------------------------------

Catkin libraries are provided by ROS packages whether you install them
from Ubuntu packages or build from source.

When your package depends on a catkin C++ library, there are usually
several kinds of dependencies which must be declared in your
``package.xml`` and ``CMakeLists.txt`` files.


package.xml
:::::::::::

Your package dependencies are declared in ``package.xml``.  If they
are missing or incorrect, you may be able to build from source and run
tests in your own workspace, but your package will not work correctly
when released to the ROS community.  Others rely on this information
to install the software they need for using your package.

The ``<build_depend>`` declares packages needed for building your
programs, including development files like headers, libraries and
configuration files.  For each build dependency, specify the ROS
package name::

  <build_depend>roscpp</build_depend>

The ``<run_depend>`` declares two different types of package
dependencies.  

One is for shared libraries, executables, Python modules, launch
scripts and other files required for running your package.

The second is for transitive build dependencies.  A common example is
when one of your dependencies provides a header file included in some
header exported from your package.  Even if your package does not use
that header when building itself, other packages depending on your
header *will* require those transitive dependencies when they are
built.

In either case, declare the dependency this way::

  <run_depend>roscpp</run_depend>

Most existing ROS packages combine their build and run-time files
within a single package.


CMakeLists.txt
::::::::::::::

CMake does not know about ``package.xml`` dependencies, although
catkin does.  For your code to compile, the ``CMakeLists.txt`` must
explicitly declare how to resolve all of your header and library
references.

Finding the library
'''''''''''''''''''

First, CMake needs to find the library.  For catkin dependencies, this
is easy::

  find_package(catkin REQUIRED COMPONENTS roscpp)

This ``find_package()`` call defines CMake variables that will be
needed later for the compile and linkedit steps.  List all additional
catkin dependencies in the same command::

  find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)

Include directories
'''''''''''''''''''

Before compiling, collect all the header paths you found earlier::

  include_directories(include ${catkin_INCLUDE_DIRS})

The ``include`` parameter is needed only if that subdirectory of your
package contains headers used to compile your programs.

Exporting interfaces
''''''''''''''''''''

You must declare the library and header packages needed by all the
interfaces you export to other ROS packages::

  catkin_package(CATKIN_DEPENDS roscpp std_msgs tf)

The ``catkin_package()`` command is only called once.  It may need
additional parameters, depending on what else your package exports.

Next steps
::::::::::

If your package also depends on non-catkin libraries provided by the
operating system, you must provide :ref:`system_library_dependencies_1`,
too.

Then, you are ready for :ref:`building_libraries_1` and
:ref:`building_executables_1`.

.. _`contributing the missing rules`: http://ros.org/doc/independent/api/rosdep/html/contributing_rules.html
.. _pkg-config: http://www.freedesktop.org/wiki/Software/pkg-config/
.. _rosdep: http://www.ros.org/wiki/rosdep
