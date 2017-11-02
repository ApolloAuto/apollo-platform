"""
Helper routines for catkin.  These are distributed inside of rosdep2
to protect catkin against future rosdep2 API updatese.  These helper
routines are assumed to run in an interactive mode with an end-user
and thus return end-user oriented error messages.

Errors are returned as arguments to raised :exc:`ValidationFailed`
exceptions.

Workflow::

    installer = get_installer(APT_INSTALLER)
    view = get_catkin_view(rosdistro_name, 'ubuntu', 'lucid')
    resolve_for_os(rosdep_key, view, installer, 'ubuntu', 'lucid')

"""

from __future__ import print_function

import os

from subprocess import Popen, PIPE, CalledProcessError

from . import create_default_installer_context
from .lookup import RosdepLookup
from .platforms.debian import APT_INSTALLER
from .platforms.osx import BREW_INSTALLER
from .platforms.pip import PIP_INSTALLER
from .platforms.redhat import YUM_INSTALLER
from .rep3 import download_targets_data
from .rosdistrohelper import get_targets
from .rospkg_loader import DEFAULT_VIEW_KEY
from .sources_list import get_sources_list_dir, DataSourceMatcher, SourcesListLoader

class ValidationFailed(Exception):
    pass

def call(command, pipe=None):
    """
    Copy of call() function from catkin-generate-debian to mimic output
    """
    working_dir = '.'
    #print('+ cd %s && ' % working_dir + ' '.join(command))
    process = Popen(command, stdout=pipe, stderr=pipe, cwd=working_dir)
    output, unused_err = process.communicate()
    retcode = process.poll()
    if retcode:
        raise CalledProcessError(retcode, command)
    if pipe:
        return output

def get_ubuntu_targets(rosdistro):
    """
    Get a list of Ubuntu distro codenames for the specified ROS
    distribution.  This method blocks on an HTTP download.

    :raises: :exc:`ValidationFailed`
    """
    targets_data = get_targets()
    legacy_targets = download_targets_data()
    if 'fuerte' in legacy_targets:
        targets_data['fuerte'] = {'ubuntu': legacy_targets['fuerte']}
    if 'electric' in legacy_targets:
        targets_data['electric'] = {'ubuntu': legacy_targets['electric']}
    return targets_data[rosdistro]['ubuntu']

def get_installer(installer_name):
    """ Expected installers APT_INSTALLER, YUM_INSTALLER, ..."""

    installer_context = create_default_installer_context()
    return installer_context.get_installer(installer_name)

def resolve_for_os(rosdep_key, view, installer, os_name, os_version):
    """
    Resolve rosdep key to dependencies.
    
    :param os_name: OS name, e.g. 'ubuntu'

    :raises: :exc:`rosdep2.ResolutionError`
    """
    d = view.lookup(rosdep_key)
    ctx = create_default_installer_context()
    os_installers = ctx.get_os_installer_keys(os_name)
    default_os_installer = ctx.get_default_os_installer_key(os_name)
    inst_key, rule = d.get_rule_for_platform(os_name, os_version, os_installers, default_os_installer)
    assert inst_key in os_installers
    return installer.resolve(rule)


def update_rosdep():
    call(('rosdep', 'update'), pipe=PIPE)


def get_catkin_view(rosdistro_name, os_name, os_version, update=True):
    """
    :raises: :exc:`ValidationFailed`
    """
    sources_list_dir = get_sources_list_dir()
    if not os.path.exists(sources_list_dir):
        raise ValidationFailed("""rosdep database is not initialized, please run:
\tsudo rosdep init
""")

    if update:
        update_rosdep()

    sources_matcher = DataSourceMatcher([rosdistro_name, os_name, os_version])
    sources_loader = SourcesListLoader.create_default(matcher=sources_matcher)
    if not (sources_loader.sources):
        raise ValidationFailed("""rosdep database does not have any sources.
Please make sure you have a valid configuration in:
\t%s
"""%(sources_list_dir))
    
    # for vestigial reasons, using the roskg loader, but we're only
    # actually using the backend db as resolution is not resource-name based
    lookup = RosdepLookup.create_from_rospkg(sources_loader=sources_loader)
    return lookup.get_rosdep_view(DEFAULT_VIEW_KEY)
