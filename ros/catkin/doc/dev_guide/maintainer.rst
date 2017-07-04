Catkin Maintainers guide
------------------------

This information is useful for maintainers of catkin itself only.

Upload catkin documentation
---------------------------

To build and upload a new version of catkin's documentation to `ros.org
<http://ros.org/doc/groovy/api/catkin/html/>`_, (this is specific
to catkin itself, and requires that you have appropriate credentials
configured)::

   git clone git://github.com/ros/catkin.git
   cd catkin/doc
   make html
   make upload
