.. _system_library_dependencies_2:

C++ system library dependencies
-------------------------------

System libraries are part of your operating system distribution.

When your package depends on a system C++ library, there are usually
several kinds of dependencies which must be declared in your
``package.xml`` and ``CMakeLists.txt`` files.


package.xml
:::::::::::

Your system package dependencies are declared in ``package.xml``.  If
they are missing or incorrect, you may be able to build from source
and run tests on your own machine, but your package will not work
correctly when released to the ROS community.  Others depend on this
information to install the software they need for using your package.

``<build_depend>``
''''''''''''''''''

This tag declares packages needed for building your programs,
including development files like headers, libraries and configuration
files.  For each build dependency, specify the corresponding rosdep_
key.  On many Linux distributions these are called "development"
packages, and their names generally end in ``-dev`` or ``-devel``,
like this::

  <build_depend>libgstreamer0.10-dev</build_depend>

Some C++ packages, like Eigen, have no run-time library, and
everything is defined in the header files::

  <build_depend>eigen</build_depend>

``<build_export_depend>``
'''''''''''''''''''''''''

If your package exports a header that includes an Eigen header like
``<Eigen/Geometry>``, then other packages that ``<build_depend>`` on
yours will need Eigen, too.  To make that work correctly, declare it
like this::

  <build_export_depend>eigen</build_export_depend>

This type of dependency mainly applies to headers and CMake
configuration files, and it typically names the "development" package::

  <build_export_depend>libgstreamer0.10-dev</build_export_depend>

``<exec_depend>``
'''''''''''''''''

The ``<exec_depend>`` is for shared libraries, executables, Python
modules, launch scripts and other files required for running your
package.  Specify the run-time rosdep key, if possible, like this::

  <exec_depend>libgstreamer0.10-0</exec_depend>

Many existing rosdep entries only name the library's "development"
package.  If no appropriate run-time package key is defined, consider
`contributing the missing rules`_ so users need not install
unnecessary files.  If you cannot provide a run-time rosdep for some
reason, you can use the "development" package for the exec dependency,
too.

``<depend>``
''''''''''''

This tag combines all the previous types of dependencies into one.  It
is not recommended for system dependencies, because it forces your
package's binary installation to depend on the "development" package,
which is not generally necessary or desirable::

  <depend>curl</depend>


CMakeLists.txt
::::::::::::::

CMake does not know about ``package.xml`` dependencies.  For your code
to compile, the ``CMakeLists.txt`` must explicitly declare how to
resolve all of your header and library references.

Finding the library
'''''''''''''''''''

First, CMake needs to find the library.  If you are lucky, someone has
already provided a *CMake module* as was done for boost.  Most boost
C++ components are fully implemented in the header files.  They do not
require a separate shared library at run-time::

  find_package(Boost REQUIRED)

But, the boost thread impementation *does* require a library, so
specify "COMPONENTS thread" if you need it::

  find_package(Boost REQUIRED COMPONENTS thread)

These ``find_package()`` calls define CMake variables that will be
needed later for the compile and linkedit steps.  While the CMake
community recommends standard names for those variables, some packages
may not follow their recommendations.  If you run across a case like
that and can't figure out what to do, get help on `answers.ros.org`_.

Sometimes, no CMake module is available, but the library's development
package provides a pkg-config_ file.  To use that, first load the
CMake ``PkgConfig`` module, then access the build flags provided by
the library::

  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GSTREAMER REQUIRED libgstreamer-0.10)

The first ``pkg_check_modules()`` parameter declares a prefix for
CMake variables like ``GSTREAMER_INCLUDE_DIRS`` and
``GSTREAMER_LIBRARIES``, later used for the compile and linkedit.  The
``REQUIRED`` argument causes configuration to fail unless the
following "module" is the base name of a pkg-config file provided by
the library, in this example ``libgstreamer-0.10.pc``.

Include directories
'''''''''''''''''''

Before compiling, collect all the header paths you found earlier using
``find_package()`` or ``pkg_check_modules()``::

  include_directories(include ${Boost_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS})

The ``include`` parameter is needed only if that subdirectory of your
package contains headers used to compile your programs.  If your
package also depends on other catkin packages, add
``${catkin_INCLUDE_DIRS}`` to the list.

Exporting interfaces
''''''''''''''''''''

The ``catkin_package()`` command is only called once.  In addition to
any other parameters, it must declare the non-catkin system library
and header packages needed by the interfaces you export to other ROS
packages::

  catkin_package(DEPENDS Boost GSTREAMER)

Make sure all these packages are also mentioned in your
``package.xml`` using a ``<build_export_depend>`` or ``<depend>`` tag.

For this to work, you must have found those dependencies earlier,
using ``find_package()`` or ``pkg_check_modules()``, and they must
define the CMake variables ``${name}_INCLUDE_DIRS`` and
``${name}_LIBRARIES``.  Note that the package name is case sensitive.
While catkin packages always use a lowercase name, other packages
might use uppercase (as ``GSTREAMER``) or mixed case (like ``Boost``).

Some packages provide variable names that do not comply with these
recommendations.  In that case, you must pass the absolute paths
explicitly as ``INCLUDE_DIRS`` and ``LIBRARIES``.

Next steps
::::::::::

At this point, you are ready for :ref:`building_libraries_2` and
:ref:`building_executables_2`.

.. _`answers.ros.org`: http://answers.ros.org
.. _`contributing the missing rules`:
   http://docs.ros.org/independent/api/rosdep/html/contributing_rules.html
.. _pkg-config: http://www.freedesktop.org/wiki/Software/pkg-config/
.. _rosdep: http://wiki.ros.org/rosdep
