.. _python_module_dependencies_2:

Python module dependencies
--------------------------

When your Python package imports other python modules, ``package.xml``
should provide a ``<exec_depend>`` with the appropriate package name.

For system dependencies, like ``python-numpy`` or ``python-yaml``, use
the corresponding rosdep name::

  <exec_depend>python-numpy</exec_depend>
  <exec_depend>python-yaml</exec_depend>

These names are usually already defined in the `rosdistro
repository`_.  If you need a module not yet defined there, please
`fork that repository and add them`_.

Several ROS infrastructure modules, like ``python-rospkg`` or
``python-rosdep`` itself, apply to multiple ROS releases and are
released independently of them.  Resolve those module dependencies
like other system packages, using the rosdep name::

  <exec_depend>python-rosdep</exec_depend>
  <exec_depend>python-rospkg</exec_depend>

When you import from another ROS Python package, like ``rospy`` or
``roslaunch``, always use the catkin package name::

  <exec_depend>roslaunch</exec_depend>
  <exec_depend>rospy</exec_depend>

ROS message or service definitions are defined as modules by ROS
packages like std_msgs_ and sensor_msgs_, used as examples here.
Although ``<exec_depend>`` is adequate when all references are in
Python, the recommended method is using a ``<depend>`` tag with the
message package name::

  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

Your ``CMakeLists.txt`` need not specify Python-only dependencies.
They are resolved automatically via ``sys.path``.

.. _`fork that repository and add them`:
   http://docs.ros.org/independent/api/rosdep/html/contributing_rules.html
.. _`rosdistro repository`:
   https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml
.. _sensor_msgs: http://wiki.ros.org/sensor_msgs
.. _std_msgs: http://wiki.ros.org/std_msgs
