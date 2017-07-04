.. _building_msgs_1:

Building messages, services or actions
--------------------------------------


package.xml
:::::::::::

Your ``package.xml`` must declare a ``<build_depend>`` on
``message_generation``, and a ``<run_depend>`` on
``message_runtime``::

  <build_depend>message_generation</build_depend>
  <run_depend>message_runtime</run_depend>

Your messages services, or actions will probably include fields
defined in other ROS messages, like std_msgs_.  Declare them like
this::

  <build_depend>std_msgs</build_depend>
  <run_depend>std_msgs</run_depend>

Both ``<build_depend>`` and ``<run_depend>`` are recommended for
message dependencies.  That example assumes ``std_msgs`` is the only
dependency.  Be sure to mention all your message package dependencies
here, and substitute them all for ``std_msgs`` in the examples that
follow.

To generate actions, add ``actionlib_msgs`` as a dependency::
  
  <build_depend>actionlib_msgs</build_depend>
  <run_depend>actionlib_msgs</run_depend>


CMakeLists.txt
::::::::::::::

For CMake, find the catkin packages for ``message_generation`` and any
messages, services or actions you depend on::

  find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

For building actions, include ``actionlib_msgs`` among the dependencies::

  find_package(catkin REQUIRED
               COMPONENTS
               actionlib_msgs
               message_generation
               std_msgs)

Next, list your message definitions::

  add_message_files(DIRECTORY msg
                    FILES
                    YourFirstMessage.msg
                    YourSecondMessage.msg
                    YourThirdMessage.msg)

Similarly, if you have a service to generate::

  add_service_files(DIRECTORY srv
                    FILES
                    YourService.srv)

To generate actions, add::

  add_action_files(DIRECTORY action
                   FILES
                   YourStartAction.action
                   YourStopAction.action)

Then, generate all your message, service and action targets with this
command::

  generate_messages(DEPENDENCIES std_msgs)

Make sure the ``catkin_package()`` command declares your message,
service and action dependencies for other packages::

  catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

A good ROS practice is to collect related messages, services and
actions into a separate package with no other API.  That simplifies
the package dependency graph.

However, you *can* provide scripts and programs with the message
package.  If you do, message generation targets need to be built
before any programs that depend on them.  Every target that directly
or indirectly uses one of your message headers must declare an
explicit dependency::

  add_dependencies(your_program ${your_package_EXPORTED_TARGETS})

If your build target also uses message or service headers imported
from other catkin packages, declare those dependencies similarly::

  add_dependencies(your_program ${catkin_EXPORTED_TARGETS})

Since catkin installs message, service and action targets
automatically, no extra ``install()`` commands are needed for them.

.. _std_msgs: http://www.ros.org/wiki/std_msgs
