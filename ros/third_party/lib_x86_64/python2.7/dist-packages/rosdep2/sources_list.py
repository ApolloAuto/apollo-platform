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

# Author Ken Conley/kwc@willowgarage.com

from __future__ import print_function

import os
import sys
import tempfile
import yaml
import hashlib
try:
    from urllib.request import urlopen
    from urllib.error import URLError
except ImportError:
    from urllib2 import urlopen
    from urllib2 import URLError
try:
    import cPickle as pickle
except ImportError:
    import pickle

from .core import InvalidData, DownloadFailure, CachePermissionError
from .gbpdistro_support import get_gbprepo_as_rosdep_data, download_gbpdistro_as_rosdep_data

try:
    import urlparse
except ImportError:
    import urllib.parse as urlparse #py3k

try:
    import httplib
except ImportError:
    import http.client as httplib  # py3k

import rospkg
import rospkg.distro

from .loader import RosdepLoader
from .rosdistrohelper import get_index, get_index_url

# default file to download with 'init' command in order to bootstrap
# rosdep
DEFAULT_SOURCES_LIST_URL = 'https://raw.github.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list'

#seconds to wait before aborting download of rosdep data
DOWNLOAD_TIMEOUT = 15.0

SOURCES_LIST_DIR = 'sources.list.d'
SOURCES_CACHE_DIR = 'sources.cache'

# name of index file for sources cache
CACHE_INDEX = 'index'

# extension for binary cache
PICKLE_CACHE_EXT = '.pickle'
SOURCE_PATH_ENV = 'ROSDEP_SOURCE_PATH'


def get_sources_list_dirs(source_list_dir):
    if SOURCE_PATH_ENV in os.environ:
        sdirs = os.environ[SOURCE_PATH_ENV].split(os.pathsep)
    else:
        sdirs = [source_list_dir]
    for p in list(sdirs):
        if not os.path.exists(p):
            sdirs.remove(p)
    return sdirs


def get_sources_list_dir():
    # base of where we read config files from
    # TODO: windows
    if 0:
        # we can't use etc/ros because environment config does not carry over under sudo
        etc_ros = rospkg.get_etc_ros_dir()
    else:
        etc_ros = '/etc/ros'
    # compute default system wide sources directory
    sys_sources_list_dir = os.path.join(etc_ros, 'rosdep', SOURCES_LIST_DIR)
    sources_list_dirs = get_sources_list_dirs(sys_sources_list_dir)
    if sources_list_dirs:
        return sources_list_dirs[0]
    else:
        return sys_sources_list_dir

def get_default_sources_list_file():
    return os.path.join(get_sources_list_dir(), '20-default.list')

def get_sources_cache_dir():
    ros_home = rospkg.get_ros_home()
    return os.path.join(ros_home, 'rosdep', SOURCES_CACHE_DIR)

# Default rosdep.yaml format.  For now this is the only valid type and
# is specified for future compatibility.
TYPE_YAML = 'yaml'
# git-buildpackage repo list
TYPE_GBPDISTRO = 'gbpdistro'
VALID_TYPES = [TYPE_YAML, TYPE_GBPDISTRO]

class DataSource(object):

    def __init__(self, type_, url, tags, origin=None):
        """
        :param type_: data source type, e.g. TYPE_YAML, TYPE_GBPDISTRO

        :param url: URL of data location.  For file resources, must
          start with the file:// scheme.  For remote resources, URL
          must include a path.

        :param tags: tags for matching data source to configurations
        :param origin: filename or other indicator of where data came from for debugging.

        :raises: :exc:`ValueError` if parameters do not validate
        """
        # validate inputs
        if not type_ in VALID_TYPES:
            raise ValueError("type must be one of [%s]"%(','.join(VALID_TYPES)))
        parsed = urlparse.urlparse(url)
        if not parsed.scheme or (parsed.scheme != 'file' and not parsed.netloc) or parsed.path in ('', '/'):
            raise ValueError("url must be a fully-specified URL with scheme, hostname, and path: %s"%(str(url)))
        if not type(tags) == list:
            raise ValueError("tags must be a list: %s"%(str(tags)))

        self.type = type_
        self.tags = tags

        self.url = url
        self.origin = origin

    def __eq__(self, other):
        return isinstance(other, DataSource) and \
               self.type == other.type and \
               self.tags == other.tags and \
               self.url == other.url and \
               self.origin == other.origin

    def __str__(self):
        if self.origin:
            return "[%s]:\n%s %s %s"%(self.origin, self.type, self.url, ' '.join(self.tags))
        else:
            return "%s %s %s"%(self.type, self.url, ' '.join(self.tags))

    def __repr__(self):
        return repr((self.type, self.url, self.tags, self.origin))

class RosDistroSource(DataSource):
    def __init__(self, distro):
        self.type = TYPE_GBPDISTRO
        self.tags = [distro]
        # In this case self.url is a list if REP-143 is being used
        self.url = get_index().distributions[distro]['distribution']
        self.origin = None

# create function we can pass in as model to parse_source_data.  The
# function emulates the CachedDataSource constructor but does the
# necessary full filepath calculation and loading of data.
def cache_data_source_loader(sources_cache_dir, verbose=False):
    def create_model(type_, uri, tags, origin=None):
        # compute the filename has from the URL
        filename = compute_filename_hash(uri)
        filepath = os.path.join(sources_cache_dir, filename)
        pickle_filepath = filepath + PICKLE_CACHE_EXT
        if os.path.exists(pickle_filepath):
            if verbose:
                print("loading cached data source:\n\t%s\n\t%s"%(uri, pickle_filepath), file=sys.stderr)
            with open(pickle_filepath, 'rb') as f:
                rosdep_data = pickle.loads(f.read())
        elif os.path.exists(filepath):
            if verbose:
                print("loading cached data source:\n\t%s\n\t%s"%(uri, filepath), file=sys.stderr)
            with open(filepath) as f:
                rosdep_data = yaml.load(f.read())
        else:
            rosdep_data = {}
        return CachedDataSource(type_, uri, tags, rosdep_data, origin=filepath)
    return create_model

class CachedDataSource(object):

    def __init__(self, type_, url, tags, rosdep_data, origin=None):
        """
        Stores data source and loaded rosdep data for that source.

        NOTE: this is not a subclass of DataSource, though it's API is
        duck-type compatible with the DataSource API.
        """
        self.source = DataSource(type_, url, tags, origin=origin)
        self.rosdep_data = rosdep_data

    def __eq__(self, other):
        try:
            return self.source == other.source and \
                   self.rosdep_data == other.rosdep_data
        except AttributeError:
            return False

    def __str__(self):
        return "%s\n%s"%(self.source, self.rosdep_data)

    def __repr__(self):
        return repr((self.type, self.url, self.tags, self.rosdep_data, self.origin))


    @property
    def type(self):
        """
        :returns: data source type
        """
        return self.source.type
    @property
    def url(self):
        """
        :returns: data source URL
        """
        return self.source.url
    @property
    def tags(self):
        """
        :returns: data source tags
        """
        return self.source.tags
    @property
    def origin(self):
        """
        :returns: data source origin, if set, or ``None``
        """
        return self.source.origin

class DataSourceMatcher(object):

    def __init__(self, tags):
        self.tags = tags

    def matches(self, rosdep_data_source):
        """
        Check if the datasource matches this configuration.

        :param rosdep_data_source: :class:`DataSource`
        """
        # all of the rosdep_data_source tags must be in our matcher tags
        return not any(set(rosdep_data_source.tags)-set(self.tags))

    @staticmethod
    def create_default(os_override=None):
        """
        Create a :class:`DataSourceMatcher` to match the current
        configuration.

        :param os_override: (os_name, os_codename) tuple to override
            OS detection
        :returns: :class:`DataSourceMatcher`
        """
        distro_name = rospkg.distro.current_distro_codename()
        if os_override is None:
            os_detect = rospkg.os_detect.OsDetect()
            os_name, os_version, os_codename = os_detect.detect_os()
        else:
            os_name, os_codename = os_override
        tags = [t for t in (distro_name, os_name, os_codename) if t]
        return DataSourceMatcher(tags)

def download_rosdep_data(url):
    """
    :raises: :exc:`DownloadFailure` If data cannot be
        retrieved (e.g. 404, bad YAML format, server down).
    """
    try:
        f = urlopen(url, timeout=DOWNLOAD_TIMEOUT)
        text = f.read()
        f.close()
        data = yaml.safe_load(text)
        if type(data) != dict:
            raise DownloadFailure('rosdep data from [%s] is not a YAML dictionary'%(url))
        return data
    except (URLError, httplib.HTTPException) as e:
        raise DownloadFailure(str(e) + ' (%s)' % url)
    except yaml.YAMLError as e:
        raise DownloadFailure(str(e))

def download_default_sources_list(url=DEFAULT_SOURCES_LIST_URL):
    """
    Download (and validate) contents of default sources list.

    :param url: override URL of default sources list file
    :return: raw sources list data, ``str``
    :raises: :exc:`InvalidData`
    :raises: :exc:`urllib2.URLError` If data cannot be
        retrieved (e.g. 404, server down).
    """
    try:
        f = urlopen(url, timeout=DOWNLOAD_TIMEOUT)
    except (URLError, httplib.HTTPException) as e:
        raise URLError(str(e) + ' (%s)' % url)
    data = f.read().decode()
    f.close()
    if not data:
        raise RuntimeError("cannot download defaults file: empty contents")
    # parse just for validation
    parse_sources_data(data)
    return data

def parse_sources_data(data, origin='<string>', model=None):
    """
    Parse sources file format (tags optional)::

      # comments and empty lines allowed
      <type> <uri> [tags]

    e.g.::

      yaml http://foo/rosdep.yaml fuerte lucid ubuntu

    If tags are specified, *all* tags must match the current
    configuration for the sources data to be used.

    :param data: data in sources file format
    :param model: model to load data into.  Defaults to :class:`DataSource`

    :returns: List of data sources, [:class:`DataSource`]
    :raises: :exc:`InvalidData`
    """
    if model is None:
        model = DataSource

    sources = []
    for line in data.split('\n'):
        line = line.strip()
        # ignore empty lines or comments
        if not line or line.startswith('#'):
            continue
        splits = line.split(' ')
        if len(splits) < 2:
            raise InvalidData("invalid line:\n%s"%(line), origin=origin)
        type_ = splits[0]
        url = splits[1]
        tags = splits[2:]
        try:
            sources.append(model(type_, url, tags, origin=origin))
        except ValueError as e:
            raise InvalidData("line:\n\t%s\n%s"%(line, e), origin=origin)
    return sources

def parse_sources_file(filepath):
    """
    Parse file on disk

    :returns: List of data sources, [:class:`DataSource`]
    :raises: :exc:`InvalidData` If any error occurs reading
        file, so an I/O error, non-existent file, or invalid format.
    """
    try:
        with open(filepath, 'r') as f:
            return parse_sources_data(f.read(), origin=filepath)
    except IOError as e:
        raise InvalidData("I/O error reading sources file: %s"%(str(e)), origin=filepath)

def parse_sources_list(sources_list_dir=None):
    """
    Parse data stored in on-disk sources list directory into a list of
    :class:`DataSource` for processing.

    :returns: List of data sources, [:class:`DataSource`]. If there is
        no sources list dir, this returns an empty list.
    :raises: :exc:`InvalidData`
    :raises: :exc:`OSError` if *sources_list_dir* cannot be read.
    :raises: :exc:`IOError` if *sources_list_dir* cannot be read.
    """
    if sources_list_dir is None:
        sources_list_dir = get_sources_list_dir()
    sources_list_dirs = get_sources_list_dirs(sources_list_dir)

    filelist = []
    for sdir in sources_list_dirs:
        filelist += sorted([os.path.join(sdir, f) for f in os.listdir(sdir) if f.endswith('.list')])
    sources_list = []
    for f in filelist:
        sources_list.extend(parse_sources_file(f))
    return sources_list


def _generate_key_from_urls(urls):
    # urls may be a list of urls or a single string
    try:
        assert isinstance(urls, (list, basestring))
    except NameError:
        assert isinstance(urls, (list, str))
    # We join the urls by the '^' character because it is not allowed in urls
    return '^'.join(urls if isinstance(urls, list) else [urls])


def update_sources_list(sources_list_dir=None, sources_cache_dir=None,
                        success_handler=None, error_handler=None):
    """
    Re-downloaded data from remote sources and store in cache.  Also
    update the cache index based on current sources.

    :param sources_list_dir: override source list directory
    :param sources_cache_dir: override sources cache directory
    :param success_handler: fn(DataSource) to call if a particular
        source loads successfully.  This hook is mainly for printing
        errors to console.
    :param error_handler: fn(DataSource, DownloadFailure) to call
        if a particular source fails.  This hook is mainly for
        printing errors to console.

    :returns: list of (`DataSource`, cache_file_path) pairs for cache
        files that were updated, ``[str]``
    :raises: :exc:`InvalidData` If any of the sources list files is invalid
    :raises: :exc:`OSError` if *sources_list_dir* cannot be read.
    :raises: :exc:`IOError` If *sources_list_dir* cannot be read or cache data cannot be written
    """
    if sources_cache_dir is None:
        sources_cache_dir = get_sources_cache_dir()

    sources = parse_sources_list(sources_list_dir=sources_list_dir)
    retval = []
    for source in list(sources):
        try:
            if source.type == TYPE_YAML:
                rosdep_data = download_rosdep_data(source.url)
            elif source.type == TYPE_GBPDISTRO:  # DEPRECATED, do not use this file. See REP137
                if not source.tags[0] in ['electric', 'fuerte']:
                    print('Ignore legacy gbpdistro "%s"' % source.tags[0])
                    sources.remove(source)
                    continue  # do not store this entry in the cache
                rosdep_data = download_gbpdistro_as_rosdep_data(source.url)
            retval.append((source, write_cache_file(sources_cache_dir, source.url, rosdep_data)))
            if success_handler is not None:
                success_handler(source)
        except DownloadFailure as e:
            if error_handler is not None:
                error_handler(source, e)

    # Additional sources for ros distros
    # In compliance with REP137 and REP143
    print('Query rosdistro index %s' % get_index_url())
    for dist_name in sorted(get_index().distributions.keys()):
        print('Add distro "%s"' % dist_name)
        rds = RosDistroSource(dist_name)
        rosdep_data = get_gbprepo_as_rosdep_data(dist_name)
        # dist_files can either be a string (single filename) or a list (list of filenames)
        dist_files = get_index().distributions[dist_name]['distribution']
        key = _generate_key_from_urls(dist_files)
        retval.append((rds, write_cache_file(sources_cache_dir, key, rosdep_data)))
        sources.append(rds)

    # Create a combined index of *all* the sources.  We do all the
    # sources regardless of failures because a cache from a previous
    # attempt may still exist.  We have to do this cache index so that
    # loads() see consistent data.
    if not os.path.exists(sources_cache_dir):
        os.makedirs(sources_cache_dir)
    cache_index = os.path.join(sources_cache_dir, CACHE_INDEX)
    data = "#autogenerated by rosdep, do not edit. use 'rosdep update' instead\n"
    for source in sources:
        url = _generate_key_from_urls(source.url)
        data += "yaml %s %s\n" % (url, ' '.join(source.tags))
    write_atomic(cache_index, data)
    # mainly for debugging and testing
    return retval

def load_cached_sources_list(sources_cache_dir=None, verbose=False):
    """
    Load cached data based on the sources list.

    :returns: list of :class:`CachedDataSource` instance with raw
        rosdep data loaded.
    :raises: :exc:`OSError` if cache cannot be read
    :raises: :exc:`IOError` if cache cannot be read
    """
    if sources_cache_dir is None:
        sources_cache_dir = get_sources_cache_dir()
    cache_index = os.path.join(sources_cache_dir, 'index')
    if not os.path.exists(cache_index):
        if verbose:
            print("no cache index present, not loading cached sources", file=sys.stderr)
        return []
    with open(cache_index, 'r') as f:
        cache_data = f.read()
    # the loader does all the work
    model = cache_data_source_loader(sources_cache_dir, verbose=verbose)
    return parse_sources_data(cache_data, origin=cache_index, model=model)


def compute_filename_hash(key_filenames):
    sha_hash = hashlib.sha1()
    if isinstance(key_filenames, list):
        for key in key_filenames:
            sha_hash.update(key.encode())
    else:
        sha_hash.update(key_filenames.encode())
    return sha_hash.hexdigest()


def write_cache_file(source_cache_d, key_filenames, rosdep_data):
    """
    :param source_cache_d: directory to write cache file to
    :param key_filenames: filename (or list of filenames) to be used in hashing
    :param rosdep_data: dictionary of data to serialize as YAML
    :returns: name of file where cache is stored
    :raises: :exc:`OSError` if cannot write to cache file/directory
    :raises: :exc:`IOError` if cannot write to cache file/directory
    """
    if not os.path.exists(source_cache_d):
        os.makedirs(source_cache_d)
    key_hash = compute_filename_hash(key_filenames)
    filepath = os.path.join(source_cache_d, key_hash)
    try:
        write_atomic(filepath + PICKLE_CACHE_EXT, pickle.dumps(rosdep_data, -1), True)
    except OSError as e:
        raise CachePermissionError("Failed to write cache file: " + str(e))
    try:
        os.unlink(filepath)
    except OSError:
        pass
    return filepath


def write_atomic(filepath, data, binary=False):
    # write data to new file
    fd, filepath_tmp = tempfile.mkstemp(prefix=os.path.basename(filepath) + '.tmp.', dir=os.path.dirname(filepath))

    if (binary):
        fmode = 'wb'
    else:
        fmode = 'w'

    with os.fdopen(fd, fmode) as f:
        f.write(data)
        f.close()

    try:
        # switch file atomically (if supported)
        os.rename(filepath_tmp, filepath)
    except OSError:
        # fall back to non-atomic operation
        try:
            os.unlink(filepath)
        except OSError:
            pass
        try:
            os.rename(filepath_tmp, filepath)
        except OSError:
            os.unlink(filepath_tmp)

class SourcesListLoader(RosdepLoader):
    """
    SourcesList loader implements the general RosdepLoader API.  This
    implementation is fairly simple as there is only one view the
    source list loader can create.  It is also a bit degenerate as it
    is not capable of mapping resource names to views, thus any
    resource-name-based API fails or returns nothing interesting.

    This loader should not be used directly; instead, it is more
    useful composed with other higher-level implementations, like the
    :class:`rosdep2.rospkg_loader.RospkgLoader`.  The general intent
    is to compose it with another loader by making all of the other
    loader's views depends on all the views in this loader.
    """

    ALL_VIEW_KEY = 'sources.list'

    def __init__(self, sources):
        """
        :param sources: cached sources list entries, [:class:`CachedDataSource`]
        """
        self.sources = sources

    @staticmethod
    def create_default(matcher=None, sources_cache_dir=None, os_override=None, verbose=False):
        """
        :param matcher: override DataSourceMatcher.  Defaults to
            DataSourceMatcher.create_default().
        :param sources_cache_dir: override location of sources cache
        """
        if matcher is None:
            matcher = DataSourceMatcher.create_default(os_override=os_override)
        if verbose:
            print("using matcher with tags [%s]"%(', '.join(matcher.tags)), file=sys.stderr)

        sources = load_cached_sources_list(sources_cache_dir=sources_cache_dir, verbose=verbose)
        if verbose:
            print("loaded %s sources"%(len(sources)), file=sys.stderr)
        sources = [x for x in sources if matcher.matches(x)]
        if verbose:
            print("%s sources match current tags"%(len(sources)), file=sys.stderr)
        return SourcesListLoader(sources)

    def load_view(self, view_name, rosdep_db, verbose=False):
        """
        Load view data into rosdep_db. If the view has already been
        loaded into rosdep_db, this method does nothing.

        :param view_name: name of ROS stack to load, ``str``
        :param rosdep_db: database to load stack data into, :class:`RosdepDatabase`

        :raises: :exc:`InvalidData`
        """
        if rosdep_db.is_loaded(view_name):
            return
        source = self.get_source(view_name)
        if verbose:
            print("loading view [%s] with sources.list loader"%(view_name), file=sys.stderr)
        view_dependencies = self.get_view_dependencies(view_name)
        rosdep_db.set_view_data(view_name, source.rosdep_data, view_dependencies, view_name)

    def get_loadable_resources(self):
        return []

    def get_loadable_views(self):
        return [x.url for x in self.sources]

    def get_view_dependencies(self, view_name):
        # use dependencies to implement precedence
        if view_name != SourcesListLoader.ALL_VIEW_KEY:
            # if the view_name matches one of our sources, return
            # empty list as none of our sources has deps.
            if any([x for x in self.sources if view_name == x.url]):
                return []

        # not one of our views, so it depends on everything we provide
        return [x.url for x in self.sources]

    def get_source(self, view_name):
        matches = [x for x in self.sources if x.url == view_name]
        if matches:
            return matches[0]
        else:
            raise rospkg.ResourceNotFound(view_name)

    def get_rosdeps(self, resource_name, implicit=True):
        """
        Always raises as SourceListLoader defines no concrete resources with rosdeps.

        :raises: :exc:`rospkg.ResourceNotFound`
        """
        raise rospkg.ResourceNotFound(resource_name)

    def get_view_key(self, resource_name):
        """
        Always raises as SourceListLoader defines no concrete resources with rosdeps.

        :returns: Name of view that *resource_name* is in, ``None`` if no associated view.
        :raises: :exc:`rospkg.ResourceNotFound` if *resource_name* cannot be found.
        """
        raise rospkg.ResourceNotFound(resource_name)
