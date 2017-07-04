^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bondpy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.7.19 (2017-03-27)
-------------------

1.7.18 (2016-10-24)
-------------------

1.7.17 (2016-03-15)
-------------------
* Queue size for a publisher `#10 <https://github.com/ros/bond_core/issues/10>`_ 
  Squash the warning.
* update maintainer
* Made code a bit more readable `#12 <https://github.com/ros/bond_core/pull/12>`_
* made local timer a member so we can cancel it during shutdown `#11 <https://github.com/ros/bond_core/pull/11>`_
* Contributors: Daniel Stonier, Esteve Fernandez, Hugo Boyer, Mikael Arguedas

1.7.16 (2014-10-30)
-------------------

1.7.15 (2014-10-28)
-------------------

1.7.14 (2014-05-08)
-------------------
* Export architecture_independent flag in package.xml `#4 <https://github.com/ros/bond_core/pull/4>`_
* bondpy: Add catkin_package call to CMakeLists.txt `#3 <https://github.com/ros/bond_core/pull/3>`_
  If catkin_package is not called, the package.xml is not installed
* Update maintainer field
* Contributors: Esteve Fernandez, Scott K Logan, Vincent Rabaud

1.7.13 (2013-08-21)
-------------------

1.7.12 (2013-06-06)
-------------------

1.7.11 (2013-03-13)
-------------------

1.7.10 (2013-01-13)
-------------------

1.7.9 (2012-12-27)
------------------
* fix wrong module for Duration
* add missing dep on rospy
* modified dep type of catkin
* Contributors: Dirk Thomas

1.7.8 (2012-12-13)
------------------
* update setup() to use generate_distutils_setup
* Contributors: Dirk Thomas

1.7.7 (2012-12-06)
------------------
* Updated url tags in package.xml's `#1 <https://github.com/ros/bond_core/pull/1>`_
* Contributors: William Woodall

1.7.6 (2012-10-30)
------------------
* fix catkin function order
* Contributors: Dirk Thomas

1.7.5 (2012-10-27)
------------------
* clean up package.xml files
* fixed python module import
* updated setup.py files
* Contributors: Dirk Thomas

1.7.4 (2012-10-06)
------------------

1.7.3 (2012-10-02 00:19)
------------------------
* fix package building issues
* Contributors: Vincent Rabaud

1.7.2 (2012-10-02 00:06)
------------------------
* add the missing catkin dependency
* Contributors: Vincent Rabaud

1.7.1 (2012-10-01 19:00)
------------------------

1.7.0 (2012-10-01 16:51)
------------------------
* catkinize the package and bump to 1.7.0 even though it is not tagged yet
* Reverting all changes that were meant to debug test failures on the build farm.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036308
* More testing bond on the build farm: being careful to shutdown bond instances between tests.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036300
* bond tests: debugging around the message publishing.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036262
* Bond tests: debugging around state transition
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036259
* bump.  Just making the build farm build again
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036258
* bond tests: typo
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036257
* Bond: debug info about status message.  Still tracking down test errors on the build farm
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036256
* More debug info for tracking down test failures in the build farm.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036255
* Modified bond's state machine to handle "alive" messages from the sibling when already dead.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036189
* Added global "bond_disable_heartbeat_timeout" parameter
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4036106
* In bond, wait_until_formed and wait_until_broken terminate when ROS shuts down.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035632
* removed wiki syntax from description
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035392
* Creating package descriptions for bondpy, bondcpp, and test_bond.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4035354
* The bond state machine more gracefully handles excessive requests to die.
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4032653
* Moving bond into common
  --HG--
  extra : convert_revision : svn%3Aeb33c2ac-9c88-4c90-87e0-44a10359b0c3/stacks/common/trunk%4032634
* Contributors: Vincent Rabaud, kwc, sglaser
