.. _system_library_dependencies_1:

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

The ``<build_depend>`` declares packages needed for building your
programs, including development files like headers, libraries and
configuration files.  For each build dependency, specify the
corresponding rosdep_ key.  On many Linux distributions these are
called "development" packages, and their names generally end in
``-dev`` or ``-devel``, like this::

  <build_depend>libgstreamer0.10-dev</build_depend>

The ``<run_depend>`` declares two different types of package
dependencies.  One is for shared libraries, executables, Python
modules, launch scripts and other files required for running your
package.  Specify the run-time rosdep key, if possible, 
like this example::

  <run_depend>libgstreamer0.10-0</run_depend>

The second type of ``<run_depend>`` is for transitive build
dependencies.  A common example is when one of your dependencies
provides a header file included in some header exported from your
package.  Even if your package does not use that header when building
itself, other packages depending on your header *will* require those
transitive dependencies when they are built.  Transitive dependencies
typically name the "development" package, instead of the run-time
package::

  <run_depend>libgstreamer0.10-dev</run_depend>

Many existing rosdep entries only name the library's "development"
package.  If no appropriate run-time package key is defined, consider
`contributing the missing rules`_ so users need not install
unnecessary files.  If you cannot provide a run-time rosdep for some
reason, you may need to use the "development" package for all
dependency declarations::

  <build_depend>curl</build_depend>
  <run_depend>curl</run_depend>

Some C++ packages, like Eigen, have no run-time library.  Everything
is defined in the header files.  As long as you do not export any
headers including them, those files are only needed at compile time.
So, you need not specify a ``<run_depend>`` in that case::

  <build_depend>eigen</build_depend>

If you *do* export headers that include Eigen headers, you also need
to declare that transitive dependency::

  <run_depend>eigen</run_depend>


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

For this to work, you must have found those dependencies earlier,
using ``find_package()`` or ``pkg_check_modules()``, and they must
define the CMake variables ``${name}_INCLUDE_DIRS`` and
``${name}_LIBRARIES``.  Note that the package name is case sensitive.
While catkin packages always use a lowercase name other packages might
use uppercase (as ``GSTREAMER``) or mixed case (as ``Boost``).

Some packages only provide variables which do not comply with these
recommendations.  Then you have to pass the absolute paths explicitly as
`INCLUDE_DIRS`` and ``LIBRARIES``.

Next steps
::::::::::

At this point, you are ready for :ref:`building_libraries_1` and
:ref:`building_executables_1`.

.. _`answers.ros.org`: http://answers.ros.org
.. _`contributing the missing rules`: http://ros.org/doc/independent/api/rosdep/html/contributing_rules.html
.. _pkg-config: http://www.freedesktop.org/wiki/Software/pkg-config/
.. _rosdep: http://www.ros.org/wiki/rosdep
