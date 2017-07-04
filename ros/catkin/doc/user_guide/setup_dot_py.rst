.. _setup_dot_py_handling:

Handling of ``setup.py``
------------------------

If your ROS package contains python modules and scripts to install,
you need to define the installation process and a way to make
the scripts accessible in the develspace.
The python ecosystem defines installation standards in the
``distutils`` or ``setuputils`` libraries. With those libraries,
packages define the installation files in a file called ``setup.py``
in the project root. The setup.py file uses Python to describe the
Python content of the stack.

We recommend to prefer distutils package over setuptools/distribute,
because with distutils we can avoid the creation of egg-info folders
in the project source folder. The setup.cfg file of distutils2 is not
supported by catkin.

Catkin allows you to specify the installation of your python files in
this setup.py and reuse some of the information in your CMakeLists.txt.

You can do so by including the line::

  catkin_python_setup()

in the CMakeLists.txt of your project.

catkin will execute setup.py with a hot-patched version of distutils
to read the arguments to set up the devel space, and execute setup.py
with suitable arguments to install to the catkin install space under
``CMAKE_INSTALL_PREFIX``.

This means you should not execute your
setup.py using::

  # DO NOT USE
  # python setup.py install

manually, as that would install to a different location, and you would
have multiple installed versions that influence each other. Using
setup.py to create a pypi package of your catkin package currently has
no support for ROS messages and services, and core ROS libraries
(e.g. rospy) are not available on pypi, so this using setup.py for
pypi is not very useful for ROS nodes.

For the develspace, the following setup.py arguments to setup() will
be used by catkin::

  from distutils.core import setup

  setup(
      version='...',
      scripts=['bin/myscript'],
      packages=['mypkg'],
      package_dir={'': 'src'}
  )

This creates relays for all scripts listed in ``scripts`` to a folder
in devel space where they can be found and executed, and also relay
packages for any package listed in ``packages``. A relay package is a
folder with an __init__.py folder and nothing else. Importing this
folder in python will execute the contents of __init__.py, which will
in turn import the original python modules in the folder in the
sourcespace using the python exec() function.

The version will be compared to that declared in package.xml, and
raise an error on mismatch.

.. note::

  If you have written non-ROS Python packages before, you have 
  probably used the ``requires`` field in the distuils setup function.
  This field, however, *has no meaning* in ROS. 
  
  All your Python
  dependencies should be specified in ``package.xml`` as e.g.
  ``<run_depend>python-numpy</run_depend>`` (for older version 1
  of package.xml) or ``<exec_depend>python-numpy</exec_depend>``
  (if you use format 2 of package.xml).
  
  Not all Python or pip packages are mapped to ROS dependencies.
  For a quick check, try running ``rosdep resolve python-mypackage``
  or ``rosdep resolve python-mypackage-pip`` if you want to 
  add a dependency on ``mypackage`` Python package. If these calls
  return error, you may want to search through the 
  `python.yaml file in rosdep`_ for similar names. If you don't 
  find the requested package, you may consider 
  `creating a Pull request to add it`_.
  
  .. _python.yaml file in rosdep: https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml
  .. _creating a Pull request to add it: http://docs.ros.org/independent/api/rosdep/html/contributing_rules.html

Using package.xml in setup.py
=============================

Writing a setup.py file without duplicating information contained in
the package.xml is possible using a catkin_pkg convenience function
like this::

  from distutils.core import setup
  from catkin_pkg.python_setup import generate_distutils_setup

  d = generate_distutils_setup(
      packages=['mypkg'],
      scripts=['bin/myscript'],
      package_dir={'': 'src'}
  )

  setup(**d)

This will parse the package.xml and also format the fields, such that
multiple authors with emails will be set nicely for setup.py, in case
one distributes to pypi.

.. note::

  ROS Users should generally not use the scripts argument, as
  in ROS, executables should be executed using rosrun rather
  than being installed to the global bin folder. One way of
  installing such python scripts is to add the following to
  the CMakeLists.txt::

    catkin_install_python(PROGRAMS scripts/myscript
      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

.. note::

  See the note in previous section about the otherwise useful
  field ``requires`` that's usually a part of distutils setup.
  *Do not use it* in ROS.


Develspace limitations
======================

For the devel space, catkin currently does not support any of the following distutils arguments:

* py_modules
* data_files
* any extention module features

From setuptools, the following args are not supported for the devel space:

* zip-safe
* entry_points

From distribute, the following args are not supported for the devel space:

* include_package_data
* exclude_package_data
* zip_safe
* entry_points
* setup_requires
* namespace_packages
* use_2to3

Those features will only work correctly for the install space.

genmsg interaction
==================

genmsg is an external catkin package that provideslanguage bindings
for ROS messages. When using the genmsg macro, ordering constraints
exist, in that case you have to invoke the macros in this order::

  project(...)
  ...
  find_package(catkin ...)
  ...
  catkin_python_setup()
  ...
  generate_messages()
  ...
  catkin_package()
