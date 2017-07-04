^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package actionlib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.5 (2016-03-14)
-------------------
* update maintainer
* Merge pull request `#42 <https://github.com/ros/actionlib/issues/42>`_ from jonbinney/python3-compat
  Python 3 compatibility changes
* More readable iteration in state name lookup
* Update syntax for exception handling
* Iterate over dictionary in python3 compatible way
* Use absolute imports for python3 compatibility
* Merge pull request `#39 <https://github.com/ros/actionlib/issues/39>`_ from clearpathrobotics/action-fixup
  Minor improvements
* Enable UI feedback for preempt-requested goal in axserver.py
* Clean up axclient.py initialization to allow starting before actionserver, requires action type passed in
* Add hashes to ServerGoalHandle and ClientGoalHandles
* Contributors: Esteve Fernandez, Jon Binney, Mikael Arguedas, Paul Bovbel

1.11.4 (2015-04-22)
-------------------
* Initialize execute_thread_ to NULL
* Contributors: Esteve Fernandez

1.11.3 (2014-12-23)
-------------------
* Increase queue sizes to match Python client publishers.
* Adjust size of client publishers in Python
* Contributors: Esteve Fernandez, Michael Ferguson

1.11.2 (2014-05-20)
-------------------
* Update python publishers to define queue_size.
* Use the correct queue for processing MessageEvents
* Contributors: Esteve Fernandez, Michael Ferguson, Nican

1.11.1 (2014-05-08)
-------------------
* Fix uninitialised execute_thread_ member pointer
* Make rostest in CMakeLists optional
* Use catkin_install_python() to install Python scripts
* Contributors: Dirk Thomas, Esteve Fernandez, Jordi Pages, Lukas Bulwahn

1.11.0 (2014-02-13)
-------------------
* replace usage of __connection_header with MessageEvent (`#20 <https://github.com/ros/actionlib/issues/20>`_)

1.10.3 (2013-08-27)
-------------------
* Merged pull request `#15 <https://github.com/ros/actionlib/issues/15>`_
  Fixes a compile issue for actionlib headers on OS X

1.10.2 (2013-08-21)
-------------------
* separating ActionServer implementation into base class and ros-publisher-based class (`#11 <https://github.com/ros/actionlib/issues/11>`_)
* support CATKIN_ENABLE_TESTING
* add isValid to ServerGoalHandle (`#14 <https://github.com/ros/actionlib/issues/14>`_)
* make operators const (`#10 <https://github.com/ros/actionlib/issues/10>`_)
* add counting of connections to avoid reconnect problem when callbacks are invoked in different order (`#7 <https://github.com/ros/actionlib/issues/7>`_)
* fix deadlock in simple_action_server.py (`#4 <https://github.com/ros/actionlib/issues/4>`_)
* fix missing runtime destination for library (`#3 <https://github.com/ros/actionlib/issues/3>`_)

1.10.1 (2013-06-06)
-------------------
* fix location of library before installation (`#1 <https://github.com/ros/actionlib/issues/1>`_)

1.10.0 (2013-04-11)
-------------------
* define DEPRECATED only if not defined already
* modified dependency type of catkin to buildtool

1.9.11 (2012-12-13)
-------------------
* first public release for Groovy

1.8.7 (2012-06-14)
------------------
* add new CommState LOST
* added more missing dependencies

1.8.6 (2012-06-05)
------------------
* added missing dependencies

1.8.5 (2012-05-31)
------------------
* make axclient work base on topic name only

1.8.4 (2012-04-05)
------------------
* add missing axserver/axclient install

1.8.3 (2012-03-15)
------------------
* fix issue with locking in action server (`#5391 <https://code.ros.org/trac/ros-pkg/ticket/5391>`_)

1.8.2 (2012-02-29)
------------------
* update to newer catkin API

1.8.1 (2012-02-21)
------------------
* fix Python packaging

1.8.0 (2012-02-07)
------------------
* separated from common stack
* converted to use catkin
