.. _catkin_library_dependencies_2:

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

``<depend>``
''''''''''''

It is generally sufficient to mention each ROS package dependency
once, like this::

  <depend>roscpp</depend>

Sometimes, you may need or want more granularity for certain
dependencies.  The following sections explain how to do that.  If in
doubt, use the ``<depend>`` tag, it's simpler.

``<build_depend>``
''''''''''''''''''

If you only use some particular dependency for building your package,
and not at execution time, you can use the ``<build_depend>`` tag.
For example, the ROS angles package only provides C++ headers and
CMake configuration files::

  <build_depend>angles</build_depend>

With this type of dependency, an installed binary of your package does
not require the angles package to be installed.

**But**, that could create a problem if your package exports a header
that includes the ``<angles/angles.h>`` header.  In that case you also
need a ``<build_export_depend>``.

``<build_export_depend>``
'''''''''''''''''''''''''

If you export a header that includes ``<angles/angles.h>``, it will be
needed by other packages that ``<build_depend>`` on yours::

  <build_export_depend>angles</build_export_depend>

This mainly applies to headers and CMake configuration files.  Library
packages referenced by libraries you export should normally specify
``<depend>``, because they are also needed at execution time.

``<exec_depend>``
'''''''''''''''''

This tag declares dependencies for shared libraries, executables,
Python modules, launch scripts and other files required when running
your package.  For example, the ROS openni_launch package provides
launch scripts, which are only needed at execution time::

  <exec_depend>openni_launch</exec_depend>


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

  find_package(catkin REQUIRED COMPONENTS angles roscpp std_msgs)

Make sure all these packages are also mentioned in your
``package.xml`` using a ``<depend>`` or ``<build_depend>`` tag.

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

  catkin_package(CATKIN_DEPENDS angles roscpp std_msgs)

Make sure all these packages are also mentioned in your
``package.xml`` using a ``<depend>`` or ``<build_export_depend>`` tag.

The ``catkin_package()`` command is only called once.  It may need
additional parameters, depending on what else your package exports.

Next steps
::::::::::

If your package also depends on non-catkin libraries provided by the
operating system, you must provide :ref:`system_library_dependencies_2`,
too.

Then, you are ready for :ref:`building_libraries_2` and
:ref:`building_executables_2`.

.. _`contributing the missing rules`:
   http://docs.ros.org/independent/api/rosdep/html/contributing_rules.html
.. _pkg-config: http://www.freedesktop.org/wiki/Software/pkg-config/
.. _rosdep: http://wiki.ros.org/rosdep
