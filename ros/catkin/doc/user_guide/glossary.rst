Glossary
--------

.. glossary::

   bloom.conf
      A file that lives on a special orphan branch `bloom` in a
      :term:`GBP repository` which contains meta-information (like
      upstream repository location and type) used when making
      releases with bloom.

   config-mode infrastructure
      Files named ``<project>Config.cmake`` and
      ``<project>Config-version.cmake`` which are used by cmake's
      ``find_package()`` in "config mode".

   dry
      a rosbuild stack or package not yet converted to catkin.

   Environment files
      The `setup.sh`, `setup.zsh` and `setup.bash` files generated
      and installed by catkin.  See also :ref:`envfiles`.

   FHS
      The Linux `Filesystem Hierarchy Standard <http://en.wikipedia.org/wiki/Filesystem_Hierarchy_Standard>`_

   generated code
      Code generated during the build process, typically by a message
      code generator package.  May or may not require compilation.

   GBP repository
      A ``git-buildpackage`` repository.  Contains released upstream
      source and the associated debian build files sufficient to
      assemble binary and source debs.  Bloom-controlled repositories
      also contain a branch ``bloom`` with meta-information.

   package
      either a not yet to catkin convert package identified by a
      manifest.xml or a catkinized package containing a package.xml.

   pkgutil
      Nifty python package:
      http://docs.python.org/library/pkgutil.html

   project
      CMake's notion of a buildable subdirectory: it contains a
      ``CMakeLists.txt`` that calls CMake's ``project()`` macro.

   stack
      unit of grouping dry packages in the old rosbuild system.  Each
      stack was packaged into a Debian package.

   static code
      Code typed in by a developer, contrast :term:`generated code`

   wet
      a ROS package already converted to catkin.
      It also applies to non-ROS code using catkin.
