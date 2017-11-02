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
Library for loading rosdep files from the ROS package/stack
filesystem.
"""

from __future__ import print_function

import catkin_pkg.package
import rospkg

from .loader import RosdepLoader

# Default view key is the view that packages that are not in stacks
# see. It is the root of all dependencies.  It is superceded by an
# explicit underlay_key.
DEFAULT_VIEW_KEY='*default*'

# Implementation details: this API was originally conceived under the
# rosdep 1 design.  It has since been retrofitted for the rosdep 2
# design, which means it is a bit overbuilt.  There really is no need
# for a notion of views for rospkg -- all rospkgs have the same view.
# It we be nice to refactor this API into something much, much
# simpler, which would probably involve merging RosPkgLoader and
# SourcesListLoader.  RosPkgLoader would provide identification of
# resources and SourcesListLoader would build a *single* view that was
# no longer resource-dependent.

class RosPkgLoader(RosdepLoader):
    
    def __init__(self, rospack=None, rosstack=None, underlay_key=None):
        """
        :param underlay_key: If set, all views loaded by this loader
            will depend on this key.
        """
        if rospack is None:
            rospack = rospkg.RosPack()
        if rosstack is None:
            rosstack = rospkg.RosStack()

        self._rospack = rospack
        self._rosstack = rosstack
        self._rosdep_yaml_cache = {}
        self._underlay_key = underlay_key
        
        # cache computed list of loadable resources
        self._loadable_resource_cache = None
        
    def load_view(self, view_name, rosdep_db, verbose=False):
        """
        Load view data into *rosdep_db*. If the view has already
        been loaded into *rosdep_db*, this method does nothing.  If
        view has no rosdep data, it will be initialized with an empty
        data map.

        :raises: :exc:`InvalidData` if view rosdep.yaml is invalid
        :raises: :exc:`rospkg.ResourceNotFound` if view cannot be located

        :returns: ``True`` if view was loaded.  ``False`` if view
          was already loaded.
        """
        if rosdep_db.is_loaded(view_name):
            return
        if not view_name in self.get_loadable_views():
            raise rospkg.ResourceNotFound(view_name)
        elif view_name == 'invalid':
            raise rospkg.ResourceNotFound("FOUND"+ view_name+str(self.get_loadable_views()))
        if verbose:
            print("loading view [%s] with rospkg loader"%(view_name))
        # chain into underlay if set
        if self._underlay_key:
            view_dependencies = [self._underlay_key]
        else:
            view_dependencies = []
        # no rospkg view has actual data
        rosdep_db.set_view_data(view_name, {}, view_dependencies, '<nodata>')

    def get_loadable_views(self):
        """
        'Views' map to ROS stack names.
        """
        return list(self._rosstack.list()) + [DEFAULT_VIEW_KEY]

    def get_loadable_resources(self):
        """
        'Resources' map to ROS packages names.
        """
        if not self._loadable_resource_cache:
            self._loadable_resource_cache = list(self._rospack.list())
        return self._loadable_resource_cache

    def get_rosdeps(self, resource_name, implicit=True):
        """
        If *resource_name* is a stack, returns an empty list.
        
        :raises: :exc:`rospkg.ResourceNotFound` if *resource_name* cannot be found.
        """
        if resource_name in self.get_loadable_resources():
            m = self._rospack.get_manifest(resource_name)
            if m.is_catkin:
                path = self._rospack.get_path(resource_name)
                pkg = catkin_pkg.package.parse_package(path)
                deps = pkg.build_depends + pkg.buildtool_depends + pkg.run_depends + pkg.test_depends
                return [d.name for d in deps]
            else:
                return self._rospack.get_rosdeps(resource_name, implicit=implicit)
        elif resource_name in self._rosstack.list():
            # stacks currently do not have rosdeps of their own, implicit or otherwise
            return []
        else:
            raise rospkg.ResourceNotFound(resource_name)

    def get_view_key(self, resource_name):
        """
        Map *resource_name* to a view key.  In rospkg, this maps the
        DEFAULT_VIEW_KEY if *resource_name* exists.

        :raises: :exc:`rospkg.ResourceNotFound`
        """
        if resource_name in self.get_loadable_resources():
            return DEFAULT_VIEW_KEY
        else:
            raise rospkg.ResourceNotFound(resource_name)
