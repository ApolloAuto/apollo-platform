Code Generation
===============

ROS messages
------------

An example for code generation is the generation of language bindings
for ROS messages. Catkin parses ``package.xml`` files for declarations
of message-generators. Those will be build first by catkin.

Other than that, it is up to packages to use genmsg by including it in
the ``CMakeLists.txt`` and calling the cmake macros defined in genmsg.

See project :ref:`genmsg <genmsg:index>`.
