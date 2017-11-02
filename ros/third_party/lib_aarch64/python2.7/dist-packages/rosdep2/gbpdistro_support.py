try:
    from urllib.request import urlopen
except ImportError:
    from urllib2 import urlopen
import yaml
try:
    import urlparse
except ImportError:
    import urllib.parse as urlparse #py3k
import os

from rospkg.os_detect import OS_DEBIAN
from rospkg.os_detect import OS_FEDORA
from rospkg.os_detect import OS_OSX
from rospkg.os_detect import OS_UBUNTU

create_default_installer_context = None
from .core import InvalidData, DownloadFailure
from .platforms.debian import APT_INSTALLER
from .platforms.osx import BREW_INSTALLER
from .platforms.redhat import YUM_INSTALLER
from .rosdistrohelper import get_targets, get_release_file, PreRep137Warning

from .rep3 import download_targets_data  # deprecated, will output warning

import warnings

#py3k
try:
    unicode
except:
    basestring = unicode = str

# location of an example gbpdistro file for reference and testing
FUERTE_GBPDISTRO_URL = 'https://raw.github.com/ros/rosdistro/' \
                     + 'master/releases/fuerte.yaml'

#seconds to wait before aborting download of gbpdistro data
DOWNLOAD_TIMEOUT = 15.0


def get_owner_name(url):
    """
    Given a gbpdistro url, returns the name of the github user in the url.

    If the url is not a valid github url it returns the default `ros`.

    This information is used to set the homebrew tap name, see:
    https://github.com/ros-infrastructure/rosdep/pull/17

    :returns: The github account in the given gbpdistro url
    """
    result = 'ros'
    try:
        parsed = urlparse.urlparse(url)
        if parsed.netloc == 'github.com':
            result = parsed.path.split('/')[1]
    except (ValueError, IndexError):
        pass
    return result


# For compatability url defaults to ''
def gbprepo_to_rosdep_data(gbpdistro_data, targets_data, url=''):
    """
    DEPRECATED: the rosdistro file format has changed according to REP137
                this function will yield a deprecation warning

    :raises: :exc:`InvalidData`
    """

    warnings.warn("deprecated: see REP137 and rosdistro", PreRep137Warning)
    # Error reporting for this isn't nearly as good as it could be
    # (e.g. doesn't separate gbpdistro vs. targets, nor provide
    # origin), but rushing this implementation a bit.
    try:
        if not type(targets_data) == dict:
            raise InvalidData("targets data must be a dict")
        if not type(gbpdistro_data) == dict:
            raise InvalidData("gbpdistro data must be a dictionary")
        if gbpdistro_data['type'] != 'gbp':
            raise InvalidData('gbpdistro must be of type "gbp"')

        # compute the default target data for the release_name
        release_name = gbpdistro_data['release-name']
        if not release_name in targets_data:
            raise InvalidData("targets file does not contain information "
                            + "for release [%s]" % (release_name))
        else:
            # take the first match
            target_data = targets_data[release_name]

        # compute the rosdep data for each repo
        rosdep_data = {}
        gbp_repos = gbpdistro_data['repositories']
        # Ensure gbp_repos is a dict
        if type(gbp_repos) != dict:
            raise InvalidData("invalid repo spec in gbpdistro data: " + str(gbp_repos)
                            + ". Invalid repositories entry, must be dict.")
        for rosdep_key, repo in gbp_repos.items():
            if type(repo) != dict:
                raise InvalidData("invalid repo spec in gbpdistro data: "
                                + str(repo))

            for pkg in repo.get('packages', {rosdep_key: None}):
                rosdep_data[pkg] = {}

                # for pkg in repo['packages']: indent the rest of the lines here.
                # Do generation for ubuntu
                rosdep_data[pkg][OS_UBUNTU] = {}
                # Do generation for empty OS X entries
                homebrew_name = '%s/%s/%s' % (get_owner_name(url),
                                              release_name, rosdep_key)
                rosdep_data[pkg][OS_OSX] = {
                    BREW_INSTALLER: {'packages': [homebrew_name]}
                }

                # - debian package name: underscores must be dashes
                deb_package_name = 'ros-%s-%s' % (release_name, pkg)
                deb_package_name = deb_package_name.replace('_', '-')

                repo_targets = repo['target'] if 'target' in repo else 'all'
                if repo_targets == 'all':
                    repo_targets = target_data

                for t in repo_targets:
                    if not isinstance(t, basestring):
                        raise InvalidData("invalid target spec: %s" % (t))
                    # rosdep_data[pkg][OS_UBUNTU][t] = {
                    rosdep_data[pkg][OS_UBUNTU][t] = {
                        APT_INSTALLER: {'packages': [deb_package_name]}
                    }

                rosdep_data[pkg]['_is_ros'] = True
        return rosdep_data
    except KeyError as e:
        raise InvalidData("Invalid GBP-distro/targets format: missing key: "
                        + str(e))


# REP137 compliant
def get_gbprepo_as_rosdep_data(gbpdistro):
    """
    :raises: :exc:`InvalidData`
    """
    distro_file = get_release_file(gbpdistro)
    ctx = create_default_installer_context()
    release_name = gbpdistro

    rosdep_data = {}
    default_installers = {}
    gbp_repos = distro_file.repositories
    for rosdep_key, repo in gbp_repos.items():
        for pkg in repo.package_names:
            rosdep_data[pkg] = {}

            # following rosdep pull #17, use env var instead of github organization name
            tap = os.environ.get('ROSDEP_HOMEBREW_TAP', 'ros')
            # Do generation for empty OS X entries
            homebrew_name = '%s/%s/%s' % (tap, release_name, rosdep_key)
            rosdep_data[pkg][OS_OSX] = {
                BREW_INSTALLER: {'packages': [homebrew_name]}
            }

            # - package name: underscores must be dashes
            package_name = 'ros-%s-%s' % (release_name, pkg)
            package_name = package_name.replace('_', '-')

            for os_name in distro_file.platforms:
                if not os_name in rosdep_data[pkg]:
                    rosdep_data[pkg][os_name] = {}
                if not os_name in default_installers:
                    default_installers[os_name] = ctx.get_default_os_installer_key(os_name)
                for os_code_name in distro_file.platforms[os_name]:
                    rosdep_data[pkg][os_name][os_code_name] = {
                        default_installers[os_name]: {'packages': [package_name]}
                    }

            rosdep_data[pkg]['_is_ros'] = True
    return rosdep_data


def download_gbpdistro_as_rosdep_data(gbpdistro_url, targets_url=None):
    """
    Download gbpdistro file from web and convert format to rosdep distro data.

    DEPRECATED: see REP137. This function will output
                (at least) one deprecation warning

    :param gbpdistro_url: url of gbpdistro file, ``str``
    :param target_url: override URL of platform targets file
    :raises: :exc:`DownloadFailure`
    :raises: :exc:`InvalidData` If targets file does not pass cursory
     validation checks.
    """
    # we can convert a gbpdistro file into rosdep data by following a
    # couple rules
    # will output a warning
    targets_data = download_targets_data(targets_url=targets_url)
    try:
        f = urlopen(gbpdistro_url, timeout=DOWNLOAD_TIMEOUT)
        text = f.read()
        f.close()
        gbpdistro_data = yaml.safe_load(text)
        # will output a warning
        return gbprepo_to_rosdep_data(gbpdistro_data,
                                      targets_data,
                                      gbpdistro_url)
    except Exception as e:
        raise DownloadFailure("Failed to download target platform data "
                            + "for gbpdistro:\n\t" + str(e))
