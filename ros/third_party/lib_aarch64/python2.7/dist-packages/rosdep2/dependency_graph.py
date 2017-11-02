# Copyright (c) 2012, Willow Garage, Inc.
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

# Author William Woodall/wjwwood@gmail.com

from collections import defaultdict

class Resolution(dict):
    """A default dictionary for use in the :class:`DependencyGraph`."""
    def __init__(self):
        super(Resolution, self).__init__()
        self['installer_key'] = None
        self['install_keys'] = []
        self['dependencies'] = []
        self['is_root'] = True

class DependencyGraph(defaultdict):
    """
    Provides a mechanism for generating a list of resolutions which preserves the dependency order.

    The :class:`DependencyGraph` inherits from a *defaultdict*, so it can be used as such to load the dependency graph data into it.
    Example:: 

        # Dependency graph:: A-B-C
        dg = DependencyGraph()
        dg['A']['installer_key'] = 'a_installer'
        dg['A']['install_keys'] = ['a']
        dg['A']['dependencies'] = ['B']
        dg['B']['installer_key'] = 'b_installer'
        dg['B']['install_keys'] = ['b']
        dg['B']['dependencies'] = ['C']
        dg['C']['installer_key'] = 'c_installer'
        dg['C']['install_keys'] = ['c']
        dg['C']['dependencies'] = []
        result = dg.get_ordered_uninstalled()

    """
    def __init__(self):
        defaultdict.__init__(self, Resolution)
    
    def detect_cycles(self, rosdep_key, traveled_keys):
        """
        Recursive function to detect cycles in the dependency graph.

        :param rosdep_key: This is the rosdep key to use as the root in the cycle exploration.
        :param traveled_keys: A list of rosdep_keys that have been traversed thus far.

        :raises: :exc:`AssertionError` if the rosdep_key is in the traveled keys, indicating a cycle has occurred.
        """
        assert rosdep_key not in traveled_keys, "A cycle in the dependency graph occurred with key `%s`."%rosdep_key
        traveled_keys.append(rosdep_key)
        for dependency in self[rosdep_key]['dependencies']:
            self.detect_cycles(dependency, traveled_keys)

    def validate(self):
        """
        Performs validations on the dependency graph, like cycle detection and invalid rosdep key detection. 

        :raises: :exc:`AssertionError` if a cycle is detected.
        :raises: :exc:`KeyError` if an invalid rosdep_key is found in the dependency graph.
        """
        for rosdep_key in self:
            # Ensure all dependencies have definitions
            # i.e.: Ensure we aren't pointing to invalid rosdep keys
            for dependency in self[rosdep_key]['dependencies']:
                if dependency not in self:
                    raise KeyError("Invalid Graph Structure: rosdep key `%s` does not exist in the dictionary of resolutions."%dependency)
                self[dependency]['is_root'] = False
        # Check each entry for cyclical dependencies
        for rosdep_key in self:
            self.detect_cycles(rosdep_key, [])

    def get_ordered_dependency_list(self):
        """
        Generates an ordered list of dependencies using the dependency graph.

        :returns: *[(installer_key, [install_keys])]*, ``[(str, [str])]``.  *installer_key* is the key
         that denotes which installed the accompanying *install_keys* are for.  *installer_key* are something 
         like ``apt`` or ``homebrew``.  *install_keys* are something like ``boost`` or ``ros-fuerte-ros_comm``.

        :raises: :exc:`AssertionError` if a cycle is detected.
        :raises: :exc:`KeyError` if an invalid rosdep_key is found in the dependency graph.
        """
        # Validate the graph
        self.validate()
        # Generate the dependency list
        dep_list = []
        for rosdep_key in self:
            if self[rosdep_key]['is_root']:
                dep_list.extend(self.__get_ordered_uninstalled(rosdep_key))
        # Make the list unique and remove empty entries
        result = []
        for item in dep_list:
            if item not in result and item[1] != []:
                result.append(item)
        # Squash the results by installer_key
        squashed_result = []
        previous_installer_key = None
        for installer_key, resolved in result:
            if previous_installer_key != installer_key:
                squashed_result.append((installer_key, []))
                previous_installer_key = installer_key
            squashed_result[-1][1].extend(resolved)
        return squashed_result

    def __get_ordered_uninstalled(self, key):
        uninstalled = []
        for dependency in self[key]['dependencies']:
            uninstalled.extend(self.__get_ordered_uninstalled(dependency))
        uninstalled.append((self[key]['installer_key'], self[key]['install_keys']))
        return uninstalled
