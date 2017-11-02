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
Underlying model of rosdep data.  The basic data model of rosdep is to
store a dictionary of data indexed by view name (i.e. ROS stack name).
This data includes a dictionary mapping rosdep dependency names to
rules and the view dependencies.

This is a lower-level representation.  Higher-level representation can
combine these rosdep dependency maps and view dependencies together
into a combined view on which queries can be made.
"""

class RosdepDatabaseEntry(object):
    """
    Stores rosdep data and metadata for a single view.
    """
    
    def __init__(self, rosdep_data, view_dependencies, origin):
        """
        :param rosdep_data: raw rosdep dictionary map for view
        :param view_dependencies: list of view dependency names
        :param origin: name of where data originated, e.g. filename
        """
        assert isinstance(rosdep_data, dict), 'RosdepDatabaseEntry() rosdep_data is not a dict: %s' % rosdep_data
        self.rosdep_data = rosdep_data
        self.view_dependencies = view_dependencies
        self.origin = origin
                
class RosdepDatabase(object):
    """
    Stores loaded rosdep data for multiple views.
    """
    
    def __init__(self):
        self._rosdep_db = {} # {view_name: RosdepDatabaseEntry}

    def is_loaded(self, view_name):
        """
        :param view_name: name of view to check, ``str``
        :returns: ``True`` if *view_name* has been loaded into this
          database.
        """
        return view_name in self._rosdep_db

    def mark_loaded(self, view_name):
        """
        If view is not already loaded, this will mark it as such.  This in effect sets the data for the view to be empty.

        :param view_name: name of view to mark as loaded
        """
        self.set_view_data(view_name, {}, [], None)
        
    def set_view_data(self, view_name, rosdep_data, view_dependencies, origin):
        """
        Set data associated with view.  This will create a new
        :class:`RosdepDatabaseEntry`.

        :param rosdep_data: rosdep data map to associated with view.
          This will be copied.
        :param origin: origin of view data, e.g. filepath of ``rosdep.yaml``
        """
        self._rosdep_db[view_name] = RosdepDatabaseEntry(rosdep_data.copy(), view_dependencies, origin)

    def get_view_names(self):
        """
        :returns: list of view names that are loaded into this database.
        """
        return self._rosdep_db.keys()
    
    def get_view_data(self, view_name):
        """
        :returns: :class:`RosdepDatabaseEntry` of given view.

        :raises: :exc:`KeyError` if no entry for *view_name*
        """
        return self._rosdep_db[view_name]
    
    def get_view_dependencies(self, view_name):
        """
        :raises: :exc:`KeyError` if *view_name* is not an entry, or if
          all of view's dependencies have not been properly loaded.
        """
        entry = self.get_view_data(view_name)
        dependencies = entry.view_dependencies[:]
        # compute full set of dependencies by iterating over
        # dependencies in reverse order and prepending.
        for s in reversed(entry.view_dependencies):
            dependencies = self.get_view_dependencies(s) + dependencies
        # make unique preserving order
        unique_deps = []
        for d in dependencies:
            if not d in unique_deps:
                unique_deps.append(d)
        return unique_deps
