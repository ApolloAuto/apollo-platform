How to build the docs
=====================

catkin provides (and uses) some plugins to `Sphinx
<http://sphinx.pocoo.org/>`_ to build documentation.  These plugins can
be used to provide a common look and feel to the generated documentation.

The first time you want to build catkin-controlled documentation (including
catkin's own documentation), you'll need to setup your environment.

Setup
-----

#. Install ``python-catkin-sphinx`` via apt-get::

       sudo apt-get install python-catkin-sphinx

#. Alternatively the package is also available via PyPi for non-Debian platforms.

Now you can build documentation for projects that use the ``ros-theme``.  For
example, to build catkin's documentation::

    git clone git://github.com/ros/catkin.git
    cd catkin/doc
    make html

Usage
-----

Ros theme
^^^^^^^^^

To use the ``ros-theme`` in your own project's documentation, add the
following line to your ``conf.py``::

    import catkin_sphinx
    html_theme_path.append(os.path.join(os.path.dirname(catkin_sphinx.__file__),
                                        'theme'))

    # Use ROS theme
    html_theme = 'ros-theme'

Shell prompts
^^^^^^^^^^^^^

If you want to document shell prompts (e.g. install instructions), use::

    extensions = extensions + ['catkin_sphinx.ShLexer']

and document your snippets using::

    .. code-block:: catkin-sh

      $ sudo pip install mystuff

Cmake macro documentation
^^^^^^^^^^^^^^^^^^^^^^^^^

If you also want to generate docs for cmake files, add::

    extensions = extensions + ['catkin_sphinx.cmake']

this will enable the sphinx "``.. cmake:macro::``" directive.
