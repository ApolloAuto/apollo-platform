# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author Ken Conley/kwc@willowgarage.com

"""
Base API for loading rosdep information by package or stack name.
This API is decoupled from the ROS packaging system to enable multiple
implementations of rosdep, including ones that don't rely on the ROS
packaging system.  This is necessary, for example, to implement a
version of rosdep that works against tarballs of released stacks.
"""

import yaml

from .core import InvalidData

ROSDEP_YAML = 'rosdep.yaml'

class RosdepLoader:
    """
    Base API for loading rosdep information by package or stack name.  
    """
    
    def load_rosdep_yaml(self, yaml_contents, origin):
        """
        Utility routine for unmarshalling rosdep data encoded as YAML.

        :param origin: origin of yaml contents (for error messages)
        :raises: :exc:`yaml.YAMLError`
        """
        try:
            return yaml.load(yaml_contents)
        except yaml.YAMLError as e:
            raise InvalidData("Invalid YAML in [%s]: %s"%(origin, e), origin=origin)

    def load_view(self, view_name, rosdep_db, verbose=False):
        """
        Load view data into rosdep_db. If the view has already been
        loaded into rosdep_db, this method does nothing.

        :param view_name: name of ROS stack to load, ``str``
        :param rosdep_db: database to load stack data into, :class:`RosdepDatabase`

        :raises: :exc:`InvalidData`
        :raises: :exc:`rospkg.ResourceNotFound` if view cannot be located
        """
        raise NotImplementedError(view_name, rosdep_db, verbose) #pychecker

    def get_loadable_resources(self):
        raise NotImplementedError()

    def get_loadable_views(self):
        raise NotImplementedError()

    def get_rosdeps(self, resource_name, implicit=True):
        """
        :raises: :exc:`rospkg.ResourceNotFound` if *resource_name* cannot be found.
        """
        raise NotImplementedError(resource_name, implicit) #pychecker
    
    def get_view_key(self, resource_name):
        """
        Map *resource_name* to a view key.  In rospkg, this maps a ROS
        package name to a ROS stack name.  If *resource_name* is a ROS
        stack name, it returns the ROS stack name.

        :returns: Name of view that *resource_name* is in, ``None`` if no associated view.
        :raises: :exc:`rospkg.ResourceNotFound` if *resource_name* cannot be found.
        """
        raise NotImplementedError(resource_name) 
