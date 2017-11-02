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

# Author Tully Foote/tfoote@willowgarage.com, Ken Conley/kwc@willowgarage.com

from __future__ import print_function

import sys
import yaml

from collections import defaultdict

from rospkg import RosPack, RosStack, ResourceNotFound

from .core import RosdepInternalError, InvalidData, rd_debug
from .model import RosdepDatabase
from .rospkg_loader import RosPkgLoader
from .dependency_graph import DependencyGraph

from .sources_list import SourcesListLoader

from . import catkin_packages

class RosdepDefinition(object):
    """
    Single rosdep dependency definition.  This data is stored as the
    raw dictionary definition for the dependency.

    See REP 111, 'Multiple Package Manager Support for Rosdep' for a
    discussion of this raw format.
    """
    
    def __init__(self, rosdep_key, data, origin="<dynamic>"):
        """
        :param rosdep_key: key/name of rosdep dependency
        :param data: raw rosdep data for a single rosdep dependency, ``dict``
        :param origin: string that indicates where data originates from (e.g. filename)
        """
        self.rosdep_key = rosdep_key
        if not isinstance(data, dict):
            raise InvalidData("rosdep data for [%s] must be a dictionary"  % (self.rosdep_key), origin=origin)
        self.data = data
        self.origin = origin

    def reverse_merge(self, new_data, origin="<dynamic>", verbose=False):
        """
        Merge two definitions together, with existing rules taking precendence.
        Definitions are merged at the os_name level, meaning that if two rules
        exist with the same os_name, the first one wins.

        :param data: raw rosdep data for a single rosdep dependency, ``dict``
        :param origin: string that indicates where this new data comes from (e.g. filename)
        """
        for os_name, rules in new_data.items():
            if os_name not in self.data:
                if verbose:
                    print("[%s] adding rules for os [%s] to [%s]"%(origin, os_name, self.rosdep_key), file=sys.stderr)
                self.data[os_name] = rules
            elif verbose:
                print("[%s] ignoring [%s] for os [%s], already loaded"%(origin, self.rosdep_key, os_name), file=sys.stderr)

    def get_rule_for_platform(self, os_name, os_version, installer_keys, default_installer_key):
        """
        Get installer_key and rule for the specified rule.  See REP 111 for precedence rules.

        :param os_name: OS name to get rule for
        :param os_version: OS version to get rule for
        :param installer_keys: Keys of installers for platform, ``[str]``
        :param default_installer_key: Default installer key for platform, ``[str]``
        :returns: (installer_key, rosdep_args_dict), ``(str, dict)``

        :raises: :exc:`ResolutionError` If no rule is available
        :raises: :exc:`InvalidData` If rule data is not valid
        """
        rosdep_key = self.rosdep_key
        data = self.data

        if type(data) != dict:
            raise InvalidData("rosdep value for [%s] must be a dictionary"%(self.rosdep_key), origin=self.origin)
        if os_name not in data:
            raise ResolutionError(rosdep_key, data, os_name, os_version, "No definition of [%s] for OS [%s]"%(rosdep_key, os_name))
        data = data[os_name]
        return_key = default_installer_key
        
        # REP 111: rosdep first interprets the key as a
        # PACKAGE_MANAGER. If this test fails, it will be interpreted
        # as an OS_VERSION_CODENAME.
        if type(data) == dict:
            for installer_key in installer_keys:
                if installer_key in data:
                    data = data[installer_key]
                    return_key = installer_key
                    break
            else:
                # data must be a dictionary, string, or list
                if type(data) == dict:
                    # check for
                    #   hardy:
                    #     apt:
                    #       stuff

                    # we've already checked for PACKAGE_MANAGER_KEY, so
                    # version key must be present here for data to be valid
                    # dictionary value.
                    if os_version not in data:
                        raise ResolutionError(rosdep_key, self.data, os_name, os_version, "No definition of [%s] for OS version [%s]"%(rosdep_key, os_version))
                    data = data[os_version]
                    if type(data) == dict:
                        for installer_key in installer_keys:
                            if installer_key in data:
                                data = data[installer_key]
                                return_key = installer_key                    
                                break

        if type(data) not in (dict, list, type('str')):
            raise InvalidData("rosdep OS definition for [%s:%s] must be a dictionary, string, or list: %s"%(self.rosdep_key, os_name, data), origin=self.origin)

        return return_key, data

    def __str__(self):
        return "%s:\n%s"%(self.origin, yaml.dump(self.data, default_flow_style=False))
    
class ResolutionError(Exception):

    def __init__(self, rosdep_key, rosdep_data, os_name, os_version, message):
        self.rosdep_key = rosdep_key
        self.rosdep_data = rosdep_data
        self.os_name = os_name
        self.os_version = os_version
        super(ResolutionError, self).__init__(message)

    def __str__(self):
        if self.rosdep_data:
            pretty_data = yaml.dump(self.rosdep_data, default_flow_style=False)
        else:
            pretty_data = '<no data>'
        return """%s
\trosdep key : %s
\tOS name    : %s
\tOS version : %s
\tData: %s"""%(self.args[0], self.rosdep_key, self.os_name, self.os_version, pretty_data)

class RosdepView(object):
    """
    View of :class:`RosdepDatabase`.  Unlike :class:`RosdepDatabase`,
    which stores :class:`RosdepDatabaseEntry` data for all stacks, a
    view merges entries for a particular stack.  This view can then be
    queries to lookup and resolve individual rosdep dependencies.
    """
    
    def __init__(self, name):
        self.name = name
        self.rosdep_defs = {} # {str: RosdepDefinition}

    def __str__(self):
        return '\n'.join(["%s: %s"%val for val in self.rosdep_defs.items()])
            
    def lookup(self, rosdep_name):
        """
        :returns: :class:`RosdepDefinition`
        :raises: :exc:`KeyError` If *rosdep_name* is not declared
        """
        return self.rosdep_defs[rosdep_name]

    def keys(self):
        """
        :returns: list of rosdep names in this view
        """
        return self.rosdep_defs.keys()
        
    def merge(self, update_entry, override=False, verbose=False):
        """
        Merge rosdep database update into main database.  Merge rules
        are first entry to declare a key wins.  There are no
        conflicts.  This rule logic is modelled after the apt sources
        list.

        :param override: Ignore first-one-wins rules and instead
            always use rules from update_entry
        """
        if verbose:
            print("view[%s]: merging from cache of [%s]"%(self.name, update_entry.origin))
        db = self.rosdep_defs

        for dep_name, dep_data in update_entry.rosdep_data.items():
            # convert data into RosdepDefinition model
            update_definition = RosdepDefinition(dep_name, dep_data, update_entry.origin)
            # First rule wins or override, no rule-merging.
            if override or not dep_name in db:
                db[dep_name] = update_definition
            elif dep_name in db:
                db[dep_name].reverse_merge(dep_data, update_entry.origin, verbose=verbose)


def prune_catkin_packages(rosdep_keys, verbose=False):
    workspace_pkgs = catkin_packages.get_workspace_packages()
    if not workspace_pkgs:
        return rosdep_keys
    for i, rosdep_key in reversed(list(enumerate(rosdep_keys))):
        if rosdep_key in workspace_pkgs:
            # If workspace packages listed (--catkin-workspace)
            # and if the rosdep_key is a package in that
            # workspace, then skip it rather than resolve it
            if verbose:
                print("rosdep key '{0}'".format(rosdep_key) +
                      " is in the catkin workspace, skipping.",
                      file=sys.stderr)
            del rosdep_keys[i]
    return rosdep_keys


def prune_skipped_packages(rosdep_keys, skipped_keys, verbose=False):
    if not skipped_keys:
        return rosdep_keys
    for i, rosdep_key in reversed(list(enumerate(rosdep_keys))):
        if rosdep_key in skipped_keys:
            # If the key is in the list of keys to explicitly skip, skip it
            if verbose:
                print("rosdep key '{0}'".format(rosdep_key) +
                      " was listed in the skipped packages, skipping.",
                      file=sys.stderr)
            del rosdep_keys[i]
    return rosdep_keys


class RosdepLookup(object):
    """
    Lookup rosdep definitions.  Provides API for most
    non-install-related commands for rosdep.

    :class:`RosdepLookup` caches data as it is loaded, so changes made
    on the filesystem will not be reflected if the rosdep information
    has already been loaded.
    """
    
    def __init__(self, rosdep_db, loader):
        """
        :param loader: Loader to use for loading rosdep data by stack
          name, ``RosdepLoader``
        :param rosdep_db: Database to load definitions into, :class:`RosdepDatabase`
        """
        self.rosdep_db = rosdep_db
        self.loader = loader
        
        self._view_cache = {} # {str: {RosdepView}}
        self._resolve_cache = {} # {str : (os_name, os_version, installer_key, resolution, dependencies)}
        
        # some APIs that deal with the entire environment save errors
        # in to self.errors instead of raising them in order to be
        # robust to single-stack faults.
        self.errors = []

        # flag for turning on printing to console
        self.verbose = False

        self.skipped_keys = []

    def get_loader(self):
        return self.loader
    
    def get_errors(self):
        """
        Retrieve error state for API calls that do not directly report
        error state.  This is the case for APIs like
        :meth:`RosdepLookup.where_defined` that are meant to be
        fault-tolerant to single-stack failures.

        :returns: List of exceptions, ``[Exception]``
        """
        return self.errors[:]
    
    def get_rosdeps(self, resource_name, implicit=True):
        """
        Get rosdeps that *resource_name* (e.g. package) requires.

        :param implicit: If ``True``, include implicit rosdep
          dependencies. Default: ``True``.

        :returns: list of rosdep names, ``[str]``
        """
        return self.loader.get_rosdeps(resource_name, implicit=implicit)

    def get_resources_that_need(self, rosdep_name):
        """
        :param rosdep_name: name of rosdep dependency
        
        :returns: list of package names that require rosdep, ``[str]``
        """
        return [k for k in self.loader.get_loadable_resources() if rosdep_name in self.get_rosdeps(k, implicit=False)]

    @staticmethod
    def create_from_rospkg(rospack=None, rosstack=None, 
                           sources_loader=None, 
                           verbose=False):
        """
        Create :class:`RosdepLookup` based on current ROS package
        environment.

        :param rospack: (optional) Override :class:`rospkg.RosPack`
          instance used to crawl ROS packages.
        :param rosstack: (optional) Override :class:`rospkg.RosStack`
          instance used to crawl ROS stacks.
        :param sources_loader: (optional) Override SourcesLoader used
            for managing sources.list data sources.
        """
        # initialize the loader
        if rospack is None:
            rospack = RosPack()
        if rosstack is None:
            rosstack = RosStack()
        if sources_loader is None:
            sources_loader = SourcesListLoader.create_default(verbose=verbose)

        rosdep_db = RosdepDatabase()

        # Use sources list to initialize rosdep_db.  Underlay has no
        # notion of specific resources, and its view keys are just the
        # individual sources it can load from.  SourcesListLoader
        # cannot do delayed evaluation of OS setting due to matcher.
        underlay_key = SourcesListLoader.ALL_VIEW_KEY
            
        # Create the rospkg loader on top of the underlay
        loader = RosPkgLoader(rospack=rospack, rosstack=rosstack,
                              underlay_key=underlay_key)

        # create our actual instance
        lookup = RosdepLookup(rosdep_db, loader)

        # load in the underlay
        lookup._load_all_views(loader=sources_loader)
        # use dependencies to implement precedence
        view_dependencies = sources_loader.get_loadable_views()
        rosdep_db.set_view_data(underlay_key, {}, view_dependencies, underlay_key)

        return lookup

    def resolve_all(self, resources, installer_context, implicit=False):
        """
        Resolve all the rosdep dependencies for *resources* using *installer_context*.

        :param resources: list of resources (e.g. packages), ``[str]``
        :param installer_context: :class:`InstallerContext`
        :param implicit: Install implicit (recursive) dependencies of
            resources.  Default ``False``.
        
        :returns: (resolutions, errors), ``([(str, [str])], {str: ResolutionError})``.  resolutions provides 
          an ordered list of resolution tuples.  A resolution tuple's first element is the installer 
          key (e.g.: apt or homebrew) and the second element is a list of opaque resolution values for that 
          installer. errors maps package names to an :exc:`ResolutionError` or :exc:`KeyError` exception.

        :raises: :exc:`RosdepInternalError` if unexpected error in constructing dependency graph
        :raises: :exc:`InvalidData` if a cycle occurs in constructing dependency graph
        """
        depend_graph = DependencyGraph()
        errors = {}
        # TODO: resolutions dictionary should be replaced with resolution model instead of mapping (undefined) keys.
        for resource_name in resources:
            try:
                rosdep_keys = self.get_rosdeps(resource_name, implicit=implicit)
                if self.verbose:
                    print("resolve_all: resource [%s] requires rosdep keys [%s]"%(resource_name, ', '.join(rosdep_keys)), file=sys.stderr)
                rosdep_keys = prune_catkin_packages(rosdep_keys, self.verbose)
                rosdep_keys = prune_skipped_packages(rosdep_keys, self.skipped_keys, self.verbose)
                for rosdep_key in rosdep_keys:
                    try:
                        installer_key, resolution, dependencies = \
                                       self.resolve(rosdep_key, resource_name, installer_context)
                        depend_graph[rosdep_key]['installer_key'] = installer_key
                        depend_graph[rosdep_key]['install_keys'] = list(resolution)
                        depend_graph[rosdep_key]['dependencies'] = list(dependencies)
                        while dependencies:
                            depend_rosdep_key = dependencies.pop()
                            # prevent infinite loop
                            if depend_rosdep_key in depend_graph:
                                continue
                            installer_key, resolution, more_dependencies = \
                                           self.resolve(depend_rosdep_key, resource_name, installer_context)
                            dependencies.extend(more_dependencies)
                            depend_graph[depend_rosdep_key]['installer_key'] = installer_key
                            depend_graph[depend_rosdep_key]['install_keys'] = list(resolution)
                            depend_graph[depend_rosdep_key]['dependencies'] = list(more_dependencies)

                    except ResolutionError as e:
                        errors[resource_name] = e
            except ResourceNotFound as e:
                errors[resource_name] = e

        try:
            # TODO: I really don't like AssertionErrors here; this should be modeled as 'CyclicGraphError' 
            # or something more explicit. No need to continue if this API errors.
            resolutions_flat = depend_graph.get_ordered_dependency_list()
        except AssertionError as e:
            raise InvalidData("cycle in dependency graph detected: %s"%(e))
        except KeyError as e:
            raise RosdepInternalError(e)

        return resolutions_flat, errors

    def resolve(self, rosdep_key, resource_name, installer_context):
        """
        Resolve a :class:`RosdepDefinition` for a particular
        os/version spec.

        :param resource_name: resource (e.g. ROS package) to resolve key within
        :param rosdep_key: rosdep key to resolve
        :param os_name: OS name to use for resolution
        :param os_version: OS name to use for resolution

        :returns: *(installer_key, resolution, dependencies)*, ``(str,
          [opaque], [str])``.  *resolution* are the system
          dependencies for the specified installer.  The value is an
          opaque list and meant to be interpreted by the
          installer. *dependencies* is a list of rosdep keys that the
          definition depends on.

        :raises: :exc:`ResolutionError` If *rosdep_key* cannot be resolved for *resource_name* in *installer_context*
        :raises: :exc:`rospkg.ResourceNotFound` if *resource_name* cannot be located
        """
        os_name, os_version = installer_context.get_os_name_and_version()

        view = self.get_rosdep_view_for_resource(resource_name)
        if view is None:
            raise ResolutionError(rosdep_key, None, os_name, os_version, "[%s] does not have a rosdep view"%(resource_name))   
        try:
            #print("KEYS", view.rosdep_defs.keys())
            definition = view.lookup(rosdep_key)
        except KeyError:
            rd_debug(view)
            raise ResolutionError(rosdep_key, None, os_name, os_version, "Cannot locate rosdep definition for [%s]"%(rosdep_key))

        # check cache: the main motivation for the cache is that
        # source rosdeps are expensive to resolve
        if rosdep_key in self._resolve_cache:
            cache_value = self._resolve_cache[rosdep_key]
            cache_os_name = cache_value[0]
            cache_os_version = cache_value[1]
            cache_view_name = cache_value[2]
            if cache_os_name == os_name and \
                   cache_os_version == os_version and \
                   cache_view_name == view.name:
                return cache_value[3:]

        # get the rosdep data for the platform
        try:
            installer_keys = installer_context.get_os_installer_keys(os_name)
            default_key = installer_context.get_default_os_installer_key(os_name)
        except KeyError:
            raise ResolutionError(rosdep_key, definition.data, os_name, os_version, "Unsupported OS [%s]"%(os_name))
        installer_key, rosdep_args_dict = definition.get_rule_for_platform(os_name, os_version, installer_keys, default_key)

        # resolve the rosdep data for the platform
        try:
            installer = installer_context.get_installer(installer_key)
        except KeyError:
            raise ResolutionError(rosdep_key, definition.data, os_name, os_version, "Unsupported installer [%s]"%(installer_key))
        resolution = installer.resolve(rosdep_args_dict)
        dependencies = installer.get_depends(rosdep_args_dict)        

        # cache value
        self._resolve_cache[rosdep_key] = os_name, os_version, view.name, installer_key, resolution, dependencies

        return installer_key, resolution, dependencies
        
    def _load_all_views(self, loader):
        """
        Load all available view keys.  In general, this is equivalent
        to loading all stacks on the package path.  If
        :exc:`InvalidData` errors occur while loading a view,
        they will be saved in the *errors* field.

        :param loader: override self.loader
        :raises: :exc:`RosdepInternalError` 
        """
        for resource_name in loader.get_loadable_views():
            try:
                self._load_view_dependencies(resource_name, loader)
            except ResourceNotFound as e:
                self.errors.append(e)
            except InvalidData as e:
                self.errors.append(e)
        
    def _load_view_dependencies(self, view_key, loader):
        """
        Initialize internal :exc:`RosdepDatabase` on demand.  Not
        thread-safe.

        :param view_key: name of view to load dependencies for.
        
        :raises: :exc:`rospkg.ResourceNotFound` If view cannot be located
        :raises: :exc:`InvalidData` if view's data is invaid
        :raises: :exc:`RosdepInternalError`
        """
        rd_debug("_load_view_dependencies[%s]"%(view_key))
        db = self.rosdep_db
        if db.is_loaded(view_key):
            return
        try:
            loader.load_view(view_key, db, verbose=self.verbose)
            entry = db.get_view_data(view_key)
            rd_debug("_load_view_dependencies[%s]: %s"%(view_key, entry.view_dependencies))            
            for d in entry.view_dependencies:
                self._load_view_dependencies(d, loader)
        except InvalidData:
            # mark view as loaded: as we are caching, the valid
            # behavior is to not attempt loading this view ever
            # again.
            db.mark_loaded(view_key)
            # re-raise
            raise
        except KeyError as e:
            raise RosdepInternalError(e)
    
    def create_rosdep_view(self, view_name, view_keys, verbose=False):
        """
        :param view_name: name of view to create
        :param view_keys: order list of view names to merge, first one wins
        :param verbose: print debugging output
        """
        # Create view and initialize with dbs from all of the
        # dependencies.
        view = RosdepView(view_name)

        db = self.rosdep_db
        for view_key in view_keys:
            db_entry = db.get_view_data(view_key)
            view.merge(db_entry, verbose=verbose)
        if verbose:
            print("View [%s], merged views:\n"%(view_name)+"\n".join([" * %s"%view_key for view_key in view_keys]), file=sys.stderr)
        return view
    
    def get_rosdep_view_for_resource(self, resource_name, verbose=False):
        """
        Get a :class:`RosdepView` for a specific ROS resource *resource_name*.
        Views can be queries to resolve rosdep keys to
        definitions.

        :param resource_name: Name of ROS resource (e.g. stack,
          package) to create view for, ``str``.

        :returns: :class:`RosdepView` for specific ROS resource
          *resource_name*, or ``None`` if no view is associated with this resource.
        
        :raises: :exc:`RosdepConflict` if view cannot be created due
          to conflict rosdep definitions.
        :raises: :exc:`rospkg.ResourceNotFound` if *view_key* cannot be located
        :raises: :exc:`RosdepInternalError` 
        """
        view_key = self.loader.get_view_key(resource_name)
        if not view_key:
            #NOTE: this may not be the right behavior and this happens
            #for packages that are not in a stack.
            return None
        return self.get_rosdep_view(view_key, verbose=verbose)
        
    def get_rosdep_view(self, view_key, verbose=False):
        """
        Get a :class:`RosdepView` associated with *view_key*.  Views
        can be queries to resolve rosdep keys to definitions.

        :param view_key: Name of rosdep view (e.g. ROS stack name), ``str``
        
        :raises: :exc:`RosdepConflict` if view cannot be created due
          to conflict rosdep definitions.
        :raises: :exc:`rospkg.ResourceNotFound` if *view_key* cannot be located
        :raises: :exc:`RosdepInternalError` 
        """
        if view_key in self._view_cache:
            return self._view_cache[view_key]

        # lazy-init
        self._load_view_dependencies(view_key, self.loader)

        # use dependencies to create view
        try:
            dependencies = self.rosdep_db.get_view_dependencies(view_key)
        except KeyError as e:
            # convert to ResourceNotFound.  This should be decoupled
            # in the future
            raise ResourceNotFound(str(e.args[0]))
        # load views in order
        view = self.create_rosdep_view(view_key, dependencies + [view_key], verbose=verbose)
        self._view_cache[view_key] = view
        return view

    def get_views_that_define(self, rosdep_name):
        """
        Locate all views that directly define *rosdep_name*.  A
        side-effect of this method is that all available rosdep files
        in the configuration will be loaded into memory.

        Error state from single-stack failures
        (e.g. :exc:`InvalidData`, :exc:`ResourceNotFound`) are
        not propagated.  Caller must check
        :meth:`RosdepLookup.get_errors` to check for single-stack
        error state.  Error state does not reset -- it accumulates.

        :param rosdep_name: name of rosdep to lookup
        :returns: list of (stack_name, origin) where rosdep is defined.

        :raises: :exc:`RosdepInternalError` 
        """
        #TODOXXX: change this to return errors object so that caller cannot ignore
        self._load_all_views(self.loader)
        db = self.rosdep_db
        retval = []
        for view_name in db.get_view_names():
            entry = db.get_view_data(view_name)
            # not much abstraction in the entry object
            if rosdep_name in entry.rosdep_data:
                retval.append((view_name, entry.origin))
            
        return retval
