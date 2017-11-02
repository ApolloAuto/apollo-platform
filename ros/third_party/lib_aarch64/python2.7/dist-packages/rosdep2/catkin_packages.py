from __future__ import print_function

import os
import sys

try:
    from catkin_pkg.packages import find_packages
except ImportError:
    print("catkin_pkg was not detected, please install it.",
          file=sys.stderr)
    sys.exit(1)

_catkin_workspace_packages = []
_catkin_packages_cache = {}


def find_catkin_packages_in(path, verbose=False):
    """
    :returns: a list of packages in a given directory
    :raises: OSError if the path doesn't exist
    """
    global _catkin_packages_cache
    if not os.path.exists(path):
        raise OSError("given path '{0}' does not exist".format(path))
    if verbose:
        print("Looking for packages in '{0}'... ".format(path),
              end='', file=sys.stderr)
    path = os.path.abspath(path)
    if path in _catkin_packages_cache:
        if verbose:
            print("found in cache.", file=sys.stderr)
        return _catkin_packages_cache[path]
    packages = find_packages(path)
    if type(packages) == dict and packages != {}:
        package_names = [package.name for package in packages.values()]
        if verbose:
            print("found " + str(len(packages)) + " packages.")
            for package in package_names:
                print("    {0}".format(package))
        _catkin_packages_cache[path] = package_names
        return package_names
    else:
        if verbose:
            print("failed to find packages.", file=sys.stderr)
        return []


def set_workspace_packages(packages):
    global _catkin_workspace_packages
    _catkin_workspace_packages = list(packages or [])


def get_workspace_packages():
    global _catkin_workspace_packages
    return _catkin_workspace_packages
