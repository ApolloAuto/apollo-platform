.. _envfiles:

Environment files
=================

Catkin creates and installs files for env-setting convenience.  In the
devel space there are ``setup.zsh`` and ``setup.bash``, these contain
variable zsh and bash specific tweaks.  They both import ``setup.sh``,
which contains bourne-shell variable settings.

.. _env.sh:
.. _setup.sh:
.. _setup.bash:
.. _setup.zsh:

.. index:: env.sh ; environment
.. index:: setup.sh ; environment
.. index:: setup.bash ; environment
.. index:: setup.zsh ; environment

``env.sh``, pointed to by the CMake variable :cmake:data:`CATKIN_ENV`
is special; it executes its arguments in the environment created by
``setup.sh`` and returns.  Any custom commands executed by CMake
should do so via this script.

.. rubric:: Environment hooks

Projects can, via the :cmake:macro:`catkin_add_env_hooks` macro, add
sh code to be executed by ``setup.sh`` (and by extension
``setup.bash`` and friends).  If you need to add things to the
environment, this is probably the place to do it.  Don't get fancy:
the contents of these scripts must be interpretable by all members of
the bourne shell family.  Be safe and ensure that ``/bin/dash`` is
okay with them.

You should avoid putting absolute paths into these environment hook
( at least in the install space). Instead use the variable
``$CATKIN_ENV_HOOK_WORKSPACE`` to point to workspace relative
resources.

**NOTE**: These environment hooks are only for variable settings,
shell aliases and functions.
