Migrating from Fuerte catkin
============================

Catkin still changes a lot, so a migration from catkin in fuerte to
catkin in groovy is necessary. Changes to catkin in Groovy are not
designed to be backwards compatible to catkin in Fuerte.

The main difference to know is that in fuerte, the atomic unit of
build was a stack declared in a ``stack.xml`` file, whereas in groovy,
``stacks`` do no exist anymore, and the atomic unit is a package
declared in a ``package.xml`` file.

If in fuerte you had a stack with several subprojects acting as
packages, this should be migrated to one metapackage and several
packages for groovy. If instead you had a stack with sources, this 
can become a single catkin package.

Metapackages are not required for packages, they may just be helpful
to define a relationship between packages, and for installing your
packages it might be easier to just give the name of a metapackage
than to list all your packages.

To update an already catkinized ROS stack from the Fuerte-version of
catkin to Groovy the following steps are necessary:

Make a metapackage for the stack
--------------------------------

The version of Catkin for ROS groovy does not support the idea of a
"stack" of packages which get wrapped up together into a Debian (or
similar) installable package. Instead, catkin packages now get built
into separate installable Debian (or similar) packages.  To maintain
compatibility with software which has a dependency on the stack name,
you need to make a new catkin package which is a "metapackage".  This
package has the same name as the stack and contains no source code,
only a list of dependencies containing all the packages which used to
be in the stack.

For example, say our old stack was named "stick" and it had 2 packages: "pickage" and "peckage".  The directories
would look like this::

  /stick/stack.xml
  /stick/CMakeLists.xml
  /stick/pickage/... (pickage files)
  /stick/peckage/... (peckage files)

The new version adds a new "stick" metapackage subdirectory under /stick, along with a package.xml and CMakeLists.txt file::

  /stick/stick/package.xml
  /stick/stick/CMakeLists.xml
  /stick/pickage/... (pickage files)
  /stick/peckage/... (peckage files)

The contents of the /stick/stick/package.xml must look like this::

  <package>
    <name>stick</name>
    <version>0.1.1</version>
    <description>Metapackage relevant to sticks.</description>
    <maintainer email="coder@example.com">Lucy Coder</maintainer>
    <license>BSD</license>

    <url type="website">http://www.example.com/stick</url>

    <buildtool_depend>catkin</buildtool_depend>

    <!-- a former stack run depends only on all its child packages -->
    <run_depend>pickage</run_depend>
    <run_depend>peckage</run_depend>

    <export>
      <metapackage/>
    </export>
  </package>

The contents of the /stick/stick/CMakeLists.txt looks like this::

  cmake_minimum_required(VERSION 2.8.3)
  project(stick)
  find_package(catkin REQUIRED)
  catkin_metapackage()

Note that the package.xml for stick has a buildtool depend on catkin. This is required
because the CMakeLists.txt for a metapackage contains a catkin macro (catkin_metapackage),
therefore catkin is required at build time. This is the only non-run_depend allowed in the
package.xml of a metapackage.

The rest of these instructions refer to changes within regular (not meta-) catkin packages.

Migrate packages
----------------

In fuerte, catkin had no strong notion of packages, only stacks, and
package manifests were at most kept for backwards compatibility with
rosbuild. In groovy, it was decided to instead drop the notion of
stacks (for metapackages), and make packages the atomic unit of build.

1. Rename ``manifest.xml`` to ``package.xml``, or create new ``package.xml``.
2. Adapt the contents of ``package.xml``

 * add a name tag
 * add a maintainer tag

3. Create/Update `CMakeLists.txt`` files

 * Instead of ``find_package(ROS ...)`` use ``find_package(catkin ...)``.

  Make sure to update the corresponding variables like ``ROS_INLCUDE_DIRS`` to ``catkin_INCLUDE_DIRS``.

  Do not search a component called ``catkin``.

* ``catkin_package(...)`` should be called near the beginning of the CMake, directly after ``find_package``-ing catkin and other dependencies.
  Unlike the old ``catkin_project()`` macro, ``catkin_package()`` doesn't need the name of the package it is part of.

* Switch to renamed catkin functions:

  * ``add_gtest`` => ``catkin_add_gtest``

    Do not use path-like string for the target name.
    The first argument must be a valid CMake target name.

  * ``add_nosetests`` => ``catkin_add_nosetests``

* Update install() invocations to use the new FHS compliant destinations (see :ref:`variables`).
  Always specify the necessary destinations explicitly.

  Specify ``DESTINATION``, ``ARCHIVE DESTINATION``, ``LIBRARY DESTINATION`` and ``RUNTIME DESTINATION`` as required.

* Remove manually ``install()`` invocations for ``stack.xml`` and ``manifest.xml`` files (this is handled by catkin automatically).

* After creating a GTest target using ``catkin_add_gtest(target ...)`` you should test for the existence of the target before trying to use it (i.e. by calling ``target_link_libraries(target ..,)``)::

  % if(TARGET target)
  %   target_link_libraries(target ...)
  % endif()

  This handles the case gracefully when GTest is not available.

CMake extra files
-----------------

CMake extra files must now work in devel space as well as in installspace.
The templates can determine the different invocation cases using the variables ``@DEVELSPACE@`` and ``@INSTALLSPACE@``.

Custom find_package() config files
----------------------------------

The ``find_package()`` config have been renamed from ``<projectname>-config.cmake.in`` to ``<ProjectName>Config.cmake.in``.
Note that the project name is no longer converted to lower case but used as-is.

Custom environment hooks
------------------------

The names of the templates for the environment hooks for devel space and installspace have been unified.
There is only one template for both.
The templates can determine the different invocation cases using the variables ``@DEVELSPACE@`` and ``@INSTALLSPACE@``.
