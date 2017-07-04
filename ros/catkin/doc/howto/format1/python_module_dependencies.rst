.. _python_module_dependencies_1:

Python module dependencies
--------------------------

When your Python package imports other python modules, ``package.xml``
should provide a ``<run_depend>`` with the appropriate package name.

For system dependencies, like ``python-numpy`` or ``python-yaml``, use
the corresponding rosdep name::

  <run_depend>python-numpy</run_depend>
  <run_depend>python-yaml</run_depend>

These names are usually already defined in the `rosdistro
repository`_.  If you need a module not yet defined there, please
`fork that repository and add them`_.

Several ROS infrastructure modules, like ``python-rospkg`` or
``python-rosdep`` itself, apply to multiple ROS releases and are
released independently of them.  Resolve those module dependencies
like other system packages, using the rosdep name::

  <run_depend>python-rosdep</run_depend>
  <run_depend>python-rospkg</run_depend>

When you import from another ROS Python package, like ``rospy`` or
``roslaunch``, always use the catkin package name::

  <run_depend>roslaunch</run_depend>
  <run_depend>rospy</run_depend>

ROS message or service definitions are similarly defined as modules by
ROS packages like std_msgs_ and sensor_msgs_, used as examples here.
For them, use both ``<build_depend>`` and ``<run_depend>`` tags with
the ROS package name::

  <build_depend>std_msgs</build_depend>
  <run_depend>std_msgs</run_depend>

  <build_depend>sensor_msgs</build_depend>
  <run_depend>sensor_msgs</run_depend>

Your ``CMakeLists.txt`` need not specify Python-only dependencies.
They are resolved automatically via ``sys.path``.

.. _`fork that repository and add them`: http://ros.org/doc/independent/api/rosdep/html/contributing_rules.html
.. _`rosdistro repository`: https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml
.. _sensor_msgs: http://www.ros.org/wiki/sensor_msgs
.. _std_msgs: http://www.ros.org/wiki/std_msgs
