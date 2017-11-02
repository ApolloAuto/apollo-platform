import copy
import os
import sys
import tarfile
import tempfile
import threading
import urllib
try:
    from urllib.request import urlopen
    from urllib.error import HTTPError
except ImportError:
    from urllib2 import urlopen
    from urllib2 import HTTPError

from .common import info
from .common import warning
from .common import error

RES_DICT = {'build': [], 'buildtool': [], 'test': [], 'run': []}
RES_TREE = {'build': {}, 'buildtool': {}, 'test': {}, 'run': {}}
CACHE_VERSION = 1

walks = {
    'FULL_WALK': {'build': ['build', 'run', 'buildtool', 'test'],
                  'run': ['build', 'run', 'buildtool', 'test'],
                  'buildtool': ['build', 'run', 'buildtool', 'test'],
                  'test': ['build', 'run', 'buildtool', 'test']},
    'SPIRAL_OF_DOOM': {'build': ['run'],
                       'run': ['buildtool'],
                       'buildtool': ['test'],
                       'test': ['build']}

}


def invert_dict(d):
    inverted = {}
    for key, value in d.iteritems():
        for v in value:
            v_keys = inverted.setdefault(v, [])
            if key not in v_keys:
                v_keys.append(key)
    return inverted


class RosDistro:
    def __init__(self, name, cache_location=None):
        self.depends_on1_cache = copy.deepcopy(RES_TREE)
        t1 = threading.Thread(target=self._construct_rosdistro_file, args=(name,))
        t2 = threading.Thread(target=self._construct_rosdistro_dependencies, args=(name, cache_location,))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

    def _construct_rosdistro_file(self, name):
        self.distro_file = RosDistroFile(name)

    def _construct_rosdistro_dependencies(self, name, cache_location):
        self.depends_file = RosDependencies(name, cache_location)

    def get_repositories(self):
        return self.distro_file.repositories

    def get_repository(self, repo):
        return self.get_repositories()[repo]

    def get_packages(self):
        return self.distro_file.packages

    def get_package(self, pkg):
        return self.get_packages()[pkg]

    def get_rosinstall(self, items, version='last_release', source='vcs'):
        rosinstall = ""
        for p in self._convert_to_pkg_list(items):
            rosinstall += p.get_rosinstall(version, source, self.distro_file.name)
        return rosinstall

    def _get_depends_on1(self, package_name):
        if package_name in self.depends_on1_cache:
            return self.depends_on1_cache[package_name]
        res = copy.deepcopy(RES_DICT)
        for pkg in self.get_packages():
            for key, depends in self._get_depends1(pkg).iteritems():
                if package_name in depends:
                    res[key].append(pkg)
        self.depends_on1_cache[package_name] = res
        return res

    def get_depends_on1(self, items):
        return self.get_depends_on(items, 1)

    def get_depends_on(self, items, depth=0, dep_dict=walks['FULL_WALK']):
        res = copy.deepcopy(RES_DICT)
        for p in self._convert_to_pkg_list(items):
            for dep_type, dep_list in res.iteritems():
                self._get_depends_on_recursive(p.name, dep_type, invert_dict(dep_dict), dep_list, depth, 1)
        return res

    def _get_depends_on_recursive(self, package_name, dep_type, dep_dict, res, depth, curr_depth):
        deps_on = self._get_depends_on1(package_name)

        # merge and recurse
        for d in deps_on[dep_type]:
            if not d in res:
                res.append(d)
                if depth == 0 or curr_depth < depth:
                    for next_dep_type in dep_dict[dep_type]:
                        self._get_depends_on_recursive(d, next_dep_type, dep_dict, res, depth, curr_depth+1)

    def _get_depends1(self, package_name):
        p = self.distro_file.packages[package_name]
        return self.depends_file.get_dependencies(p, self.distro_file.name)

    def get_depends1(self, items):
        return self.get_depends(items, 1)

    def get_depends(self, items, depth=0, dep_dict=walks['FULL_WALK']):
        res = copy.deepcopy(RES_DICT)
        for p in self._convert_to_pkg_list(items):
            for dep_type, dep_list in res.iteritems():
                self._get_depends_recursive(p.name, dep_type, dep_dict, dep_list, depth, 1)
        return res

    def _get_depends_recursive(self, package_name, dep_type, dep_dict, res, depth, curr_depth):
        deps1 = self._get_depends1(package_name)

        # merge and recurse
        for d in deps1[dep_type]:
            if not d in res:
                res.append(d)
                if depth == 0 or curr_depth < depth:
                    if d in self.get_packages():  # recurse on packages only
                        for next_dep_type in dep_dict[dep_type]:
                            self._get_depends_recursive(d, next_dep_type, dep_dict, res, depth, curr_depth+1)

    def _convert_to_pkg_list(self, items):
        if type(items) != list:
            items = [items]
        pkgs = []
        for i in items:
            if i in self.distro_file.repositories:
                for p in self.distro_file.repositories[i].packages:
                    if not p in pkgs:
                        pkgs.append(p)
            elif i in self.distro_file.packages:
                if not self.distro_file.packages[i] in pkgs:
                    pkgs.append(self.distro_file.packages[i])
            else:
                raise RuntimeError("!!! {0} is not a package name nor a repository name".format(i))
        return pkgs


class RosDistroFile:
    def __init__(self, name):
        self.packages = {}
        self.repositories = {}
        self.name = name

        # parse ros distro file
        distro_url = urlopen('https://raw.github.com/ros/rosdistro/master/releases/%s.yaml' % name)
        distro = yaml.load(distro_url.read())['repositories']

        # loop over all repo's
        for repo_name, data in distro.iteritems():
            repo = RosRepository(repo_name, data['version'], data['url'])
            self.repositories[repo_name] = repo
            if 'packages' not in data:  # support unary disto's
                data['packages'] = {repo_name: ''}

            # loop over all packages
            for pkg_name in data['packages'].keys():
                pkg = RosPackage(pkg_name, repo)
                repo.packages.append(pkg)
                self.packages[pkg_name] = pkg


class RosRepository:
    def __init__(self, name, version, url):
        self.name = name
        self.version = version
        self.url = url
        self.packages = []

    def get_rosinstall(self, version, source):
        return "\n".join([p.get_rosinstall(version, source) for p in self.packages])


class RosPackage:
    def __init__(self, name, repository):
        self.name = name
        self.repository = repository
        self._package_xmls = {}
        self._release_tags = {}

    def _fetch_package_xml(self, rosdistro):
        repo = self.repository
        if 'github.com' in repo.url:
            url = repo.url
            upstream_version = repo.version.split('-')[0]
            release_tag = 'release/{0}/{1}'.format(self.name, upstream_version)
            url = url.replace('.git', '/{0}/package.xml'.format(release_tag))
            url = url.replace('git://', 'https://')
            url = url.replace('https://', 'https://raw.')
            try:
                try:
                    package_xml = urlopen(url).read()
                except Exception as e:
                    msg = "Failed to read package.xml file from url '{0}': {1}".format(url, e)
                    warning(msg)
                    url = repo.url
                    release_tag = 'release/{0}/{1}/{2}'.format(rosdistro, self.name, repo.version)
                    tail = '/{0}/package.xml'.format(release_tag)
                    url = url.replace('.git', tail)
                    url = url.replace('git://', 'https://')
                    url = url.replace('https://', 'https://raw.')
                    info("Trying to read from url '{0}' instead".format(url))
                    package_xml = urlopen(url).read()
            except Exception as e:
                msg += '\nAND\n'
                msg += "Failed to read package.xml file from url '{0}': {1}".format(url, e)
                raise RuntimeError(msg)
            self._package_xmls[rosdistro] = package_xml
            self._release_tags[rosdistro] = release_tag
            return package_xml, release_tag
        else:
            raise Exception("Non-github repositories are net yet supported by the rosdistro tool")

    def get_package_xml(self, rosdistro):
        if rosdistro not in self._package_xmls:
            self._fetch_package_xml(rosdistro)
        return self._package_xmls[rosdistro]

    def get_release_tag(self, rosdistro):
        if rosdistro not in self._release_tags:
            self._fetch_package_xml(rosdistro)
        return self._release_tags[rosdistro]

    def get_rosinstall(self, version, source, rosdistro):
        # can't get last release of unreleased repository
        if version == 'last_release' and not self.repository.version:
            raise RuntimeError("Can't get the last release of unreleased repository {0}".format(self.repository.name))

        # set specific version of last release of needed
        if version == 'last_release':
            version = self.repository.version.split('-')[0]

        # generate the rosinstall file
        release_tag = self.get_release_tag(rosdistro)
        if version == 'master':
            return yaml.dump([{
                'git': {
                    'local-name': self.name,
                    'uri': self.repository.url,
                    'version': '/'.join(release_tag.split('/')[:-1])
                }}],
                default_style=False)
        else:
            if source == 'vcs':
                return yaml.safe_dump([{
                    'git': {
                        'local-name': self.name,
                        'uri': self.repository.url,
                        'version': release_tag
                    }}],
                    default_style=False)
            elif source == 'tar':
                uri = self.repository.url
                uri = uri.replace('git://', 'https://')
                uri = uri.replace('.git', '/archive/{0}.tar.gz'.format(release_tag))
                return yaml.safe_dump([{
                    'tar': {
                        'local-name': self.name,
                        'uri': uri,
                        'version': '{0}-release-{1}'.format(self.repository.name, release_tag.replace('/', '-'))
                    }}],
                    default_style=False)
            else:
                raise RuntimeError("Invalid source type {0}".format(source))


class RosDependencies:
    def __init__(self, name, cache_location):
        # url's
        self.file_name = '%s-dependencies.yaml' % name
        if cache_location:
            self.local_url = os.path.join(cache_location, self.file_name)
        else:
            from rospkg import environment
            self.local_url = os.path.join(environment.get_ros_home(), self.file_name)
        self.server_url = 'http://www.ros.org/rosdistro/%s-dependencies.tar.gz' % name
        self.dependencies = {}

        # initialize with the local or server cache
        deps = self._read_local_cache()
        if deps == {}:
            deps = self._read_server_cache()
        for key, value in deps.iteritems():
            self.dependencies[key] = value
        if self.cache == 'server':
            self._write_local_cache()

    def get_dependencies(self, package, rosdistro):
        repo = package.repository
        # support unreleased stacks
        if not repo.version:
            return copy.deepcopy(RES_DICT)

        key = '%s?%s?%s' % (repo.name, repo.version, package.name)

        # check in memory first
        if key in self.dependencies:
            return self.dependencies[key]

        # read server cache if needed
        if self.cache != 'server':
            deps = self._read_server_cache()
            for key, value in deps.iteritems():
                self.dependencies[key] = value
            self._write_local_cache()
            if key in self.dependencies:
                return self.dependencies[key]

        # retrieve dependencies
        deps = retrieve_dependencies(package.get_package_xml(rosdistro))
        self.dependencies[key] = deps
        self._write_local_cache()
        return deps

    def _read_server_cache(self):
        self.cache = 'server'
        try:
            resp = urlopen(self.server_url)
        except HTTPError as ex:
            warning("Failed to read server cache: %s" % ex)
            return {}
        with tempfile.NamedTemporaryFile('w') as fh:
            fh.write(resp.read())
            fh.flush()

            tar = tarfile.open(fh.name, 'r')
        data = tar.extractfile(self.file_name)
        deps = yaml.load(data.read())
        if not deps \
           or not 'cache_version' in deps \
           or deps['cache_version'] != CACHE_VERSION \
           or not 'repositories' in deps:
            raise
        return deps['repositories']

    def _read_local_cache(self):
        try:
            self.cache = 'local'
            with open(self.local_url) as f:
                deps = yaml.safe_load(f.read())
                if not deps \
                   or not 'cache_version' in deps \
                   or deps['cache_version'] != CACHE_VERSION \
                   or not 'repositories' in deps:
                    raise
                return deps['repositories']
        except Exception:
            return {}

    def _write_local_cache(self):
        try:
            try:
                os.makedirs(os.path.dirname(self.local_url))
            except:
                pass
            with open(self.local_url, 'w') as f:
                yaml.dump({'cache_version': CACHE_VERSION,
                           'repositories': self.dependencies},
                          f)
        except Exception as ex:
            error("Failed to write local dependency cache to %s: %s" % (self.local_url, ex))


def retrieve_dependencies(package_xml):
    try:
        return get_package_dependencies(package_xml)
    except Exception:
        raise RuntimeError("Failed to get dependencies from package_xml:\n```\n{0}\n```".format(package_xml))


def get_package_dependencies(package_xml):
    if not os.path.abspath("/usr/lib/pymodules/python2.7") in sys.path:
        sys.path.append("/usr/lib/pymodules/python2.7")
    from catkin_pkg import package as catkin_pkg

    pkg = catkin_pkg.parse_package_string(package_xml)
    depends1 = {'build': [d.name for d in pkg.build_depends],
                'buildtool':  [d.name for d in pkg.buildtool_depends],
                'test':  [d.name for d in pkg.test_depends],
                'run':  [d.name for d in pkg.run_depends]}
    return depends1
