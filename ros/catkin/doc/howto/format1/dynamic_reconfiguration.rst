.. _dynamic_reconfiguration_1:

Dynamic reconfiguration
-----------------------

Configure script
::::::::::::::::

Dynamic reconfiguration requires you to provide one or more simple
Python scripts that declare the names, types, values, and ranges of
the parameters you wish to configure dynamically.  See the tutorials_
for details.

Catkin configure scripts differ from the earlier rosbuild interface.
First, remove any reference to roslib_, which is not needed with
catkin::

  import roslib; roslib.load_manifest(PACKAGE)

Then, be sure to use the catkin version of the parameter generator::

  from dynamic_reconfigure.parameter_generator_catkin import *

The remainder of your scripts need not change.

package.xml
:::::::::::

You need to declare both build and run-time dependencies on
dynamic_reconfigure_::

  <build_depend>dynamic_reconfigure</build_depend>
  <run_depend>dynamic_reconfigure</run_depend>

CMakeLists.txt
::::::::::::::

Be sure to include dynamic_reconfigure_ among your catkin package
components::

  find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure ...)

Generate the reconfigure options, listing all your node's ``.cfg``
scripts::

  generate_dynamic_reconfigure_options(cfg/YourNode.cfg)

Since you probably have build targets using the generated header, add
this to ensure it gets built before any targets needing them::

  add_dependencies(your_program ${your_package_EXPORTED_TARGETS})
  add_dependencies(your_library ${your_package_EXPORTED_TARGETS})

Finally, include dynamic_reconfigure_ in the CATKIN_DEPENDS exports
list for your package::

  catkin_package(CATKIN_DEPENDS dynamic_reconfigure ...)

.. _dynamic_reconfigure: http://ros.org/wiki/dynamic_reconfigure
.. _roslib: http://ros.org/wiki/roslib
.. _tutorials: http://ros.org/wiki/dynamic_reconfigure/Tutorials
