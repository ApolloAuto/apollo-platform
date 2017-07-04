.. _installing_other_2:

Installing other files
----------------------

Sometimes your package needs to install additional files, like
`roslaunch scripts`_ or parameter settings.

In most cases these data are platform-independent, so install them
within your package's share directory::

  install(FILES your_data your_parameters
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

This example installs everything in your **launch/** subdirectory::

  install(DIRECTORY launch/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
          PATTERN ".svn" EXCLUDE)

The ``PATTERN ".svn" EXCLUDE`` is only needed if you use a Subversion_
repository.  For other types of repositories, it can be omitted.

.. _`roslaunch scripts`: http://wiki.ros.org/roslaunch/XML
.. _Subversion: http://subversion.apache.org/
