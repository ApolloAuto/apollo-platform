.. _installing_python_1:

Installing Python scripts and modules
-------------------------------------

Even if ``your_package`` only contains Python code, it still needs a
catkin ``CMakeLists.txt`` to install executable scripts and to export
modules so they can be imported in other ROS packages.


Scripts
:::::::

ROS executables are installed in a per-package directory, not the
distributions's global ``bin/`` directory.  They are accessible to
rosrun_ and roslaunch_, without cluttering up the shell's ``$PATH``,
and their names only need to be unique within each package.  There are
only a few core ROS commands like ``rosrun`` and ``roslaunch`` that
install in the global ``bin/`` directory.

Standard ROS practice is to place all executable Python programs in a
package subdirectory named ``nodes/`` or ``scripts/``.  Their usage is
the same, the two names distinguish ROS nodes from other executable
Python scripts.  To keep the user API clean,
executable script names generally do not include a ``.py`` suffix.
Your ``CMakeLists.txt`` should install all the scripts explictly
using the special install function ``catkin_install_python``.
This will make sure that shebang lines are updated to use the
specific Python version used at configure time::

  catkin_install_python(PROGRAMS nodes/your_node scripts/another_script
                        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

Another good practice is to keep executable scripts very short,
placing most of the code in a module which the script imports and then
invokes::

  #! /usr/bin/env python
  import your_package.main
  if __name__ == '__main__':
      your_package.main()


Modules
:::::::

Standard ROS practice is to place Python modules under the
``src/your_package`` subdirectory, making the top-level module name
the same as your package.  Python requires that directory to have an
``__init__.py`` file, too.

.. note::

  With rosbuild, it was possible to place Python modules directly
  within ``src/``.  That violated Python setup conventions, and catkin
  does not allow it.  If you need to define a module named
  ``your_package``, place its code in ``src/your_package/__init__.py``
  or import its public symbols there.

Catkin installs Python packages using a variant of the standard Python
``setup.py`` script.  Assuming your modules use the standard ROS
layout, it looks like this::

  ## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

  from distutils.core import setup
  from catkin_pkg.python_setup import generate_distutils_setup
  
  # fetch values from package.xml
  setup_args = generate_distutils_setup(
      packages=['your_package'],
      package_dir={'': 'src'})
  
  setup(**setup_args)

This ``setup.py`` is only for use with catkin. Remember *not* to
invoke it yourself.

Put that script in the top directory of your package, and add this to
your ``CMakeLists.txt``::

  catkin_python_setup()

That takes care of installing your Python modules.  Never use it to
install executable scripts.  Use the ``catkin_install_python()``
command shown above.

.. _roslaunch: http://wiki.ros.org/roslaunch
.. _rosrun: http://wiki.ros.org/rosrun
.. _Subversion: http://subversion.apache.org/
