# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Library for parsing package.xml and providing an object
representation.
"""

from __future__ import print_function

from copy import deepcopy
import os
import re
import sys
import xml.dom.minidom as dom

PACKAGE_MANIFEST_FILENAME = 'package.xml'


class Package(object):
    """
    Object representation of a package manifest file
    """
    __slots__ = [
        'package_format',
        'name',
        'version',
        'version_abi',
        'description',
        'maintainers',
        'licenses',
        'urls',
        'authors',
        'build_depends',
        'buildtool_depends',
        'build_export_depends',
        'buildtool_export_depends',
        'exec_depends',
        'test_depends',
        'doc_depends',
        'conflicts',
        'replaces',
        'exports',
        'filename'
    ]

    def __init__(self, filename=None, **kwargs):
        """
        :param filename: location of package.xml.  Necessary if
          converting ``${prefix}`` in ``<export>`` values, ``str``.
        """
        # initialize all slots ending with "s" with lists, all other with plain values
        for attr in self.__slots__:
            if attr.endswith('s'):
                value = list(kwargs[attr]) if attr in kwargs else []
                setattr(self, attr, value)
            else:
                value = kwargs[attr] if attr in kwargs else None
                setattr(self, attr, value)
        if 'depends' in kwargs:
            for d in kwargs['depends']:
                for slot in [self.build_depends, self.build_export_depends, self.exec_depends]:
                    if d not in slot:
                        slot.append(deepcopy(d))
            del kwargs['depends']
        if 'run_depends' in kwargs:
            for d in kwargs['run_depends']:
                for slot in [self.build_export_depends, self.exec_depends]:
                    if d not in slot:
                        slot.append(deepcopy(d))
            del kwargs['run_depends']
        self.filename = filename
        # verify that no unknown keywords are passed
        unknown = set(kwargs.keys()).difference(self.__slots__)
        if unknown:
            raise TypeError('Unknown properties: %s' % ', '.join(unknown))

    def __getattr__(self, name):
        if name == 'run_depends':
            # merge different dependencies if they are not exactly equal
            # potentially having the same dependency name multiple times with different attributes
            run_depends = []
            [run_depends.append(deepcopy(d)) for d in self.exec_depends + self.build_export_depends if d not in run_depends]
            return run_depends
        raise AttributeError(name)

    def __getitem__(self, key):
        if key in self.__slots__ + ['run_depends']:
            return getattr(self, key)
        raise KeyError('Unknown key "%s"' % key)

    def __iter__(self):
        for slot in self.__slots__:
            yield slot

    def __str__(self):
        data = {}
        for attr in self.__slots__:
            data[attr] = getattr(self, attr)
        return str(data)

    def has_buildtool_depend_on_catkin(self):
        """
        Returns True if this Package buildtool depends on catkin, otherwise False

        :returns: True if the given package buildtool depends on catkin
        :rtype: bool
        """
        return 'catkin' in [d.name for d in self.buildtool_depends]

    def has_invalid_metapackage_dependencies(self):
        """
        Returns True if this package has invalid dependencies for a metapackage

        This is defined by REP-0127 as any non-run_depends dependencies other then a buildtool_depend on catkin.

        :returns: True if the given package has any invalid dependencies, otherwise False
        :rtype: bool
        """
        buildtool_depends = [d.name for d in self.buildtool_depends if d.name != 'catkin']
        return len(self.build_depends + buildtool_depends + self.test_depends) > 0

    def is_metapackage(self):
        """
        Returns True if this pacakge is a metapackage, otherwise False

        :returns: True if metapackage, else False
        :rtype: bool
        """
        return 'metapackage' in [e.tagname for e in self.exports]

    def validate(self, warnings=None):
        """
        makes sure all standards for packages are met
        :param package: Package to check
        :param warnings: Print warnings if None or return them in the given list
        :raises InvalidPackage: in case validation fails
        """
        errors = []
        new_warnings = []

        if self.package_format:
            if not re.match('^[1-9][0-9]*$', str(self.package_format)):
                errors.append('The "format" attribute of the package must contain a positive integer if present')

        if not self.name:
            errors.append('Package name must not be empty')
        # accepting upper case letters and hyphens only for backward compatibility
        if not re.match('^[a-zA-Z0-9][a-zA-Z0-9_-]*$', self.name):
            errors.append('Package name "%s" does not follow naming conventions' % self.name)
        elif not re.match('^[a-z][a-z0-9_]*$', self.name):
            new_warnings.append('Package name "%s" does not follow the naming conventions. It should start with a lower case letter and only contain lower case letters, digits and underscores.' % self.name)

        if not self.version:
            errors.append('Package version must not be empty')
        elif not re.match('^[0-9]+\.[0-9]+\.[0-9]+$', self.version):
            errors.append('Package version "%s" does not follow version conventions' % self.version)
        elif not re.match('^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$', self.version):
            new_warnings.append('Package "%s" does not follow the version conventions. It should not contain leading zeros (unless the number is 0).' % self.name)

        if not self.description:
            errors.append('Package description must not be empty')

        if not self.maintainers:
            errors.append('Package must declare at least one maintainer')
        for maintainer in self.maintainers:
            try:
                maintainer.validate()
            except InvalidPackage as e:
                errors.append(str(e))
            if not maintainer.email:
                errors.append('Maintainers must have an email address')

        if not self.licenses:
            errors.append('The package node must contain at least one "license" tag')
        if [l for l in self.licenses if not l.strip()]:
            errors.append('The license tag must neither be empty nor only contain whitespaces')

        if self.authors is not None:
            for author in self.authors:
                try:
                    author.validate()
                except InvalidPackage as e:
                    errors.append(str(e))

        dep_types = {
            'build': self.build_depends,
            'buildtool': self.buildtool_depends,
            'build_export': self.build_export_depends,
            'buildtool_export': self.buildtool_export_depends,
            'exec': self.exec_depends,
            'test': self.test_depends,
            'doc': self.doc_depends
        }
        for dep_type, depends in dep_types.items():
            for depend in depends:
                if depend.name == self.name:
                    errors.append('The package must not "%s_depend" on a package with the same name as this package' % dep_type)

        if self.is_metapackage():
            if not self.has_buildtool_depend_on_catkin():
                # TODO escalate to error in the future, or use metapackage.validate_metapackage
                new_warnings.append('Metapackage "%s" must buildtool_depend on catkin.' % self.name)
            if self.has_invalid_metapackage_dependencies():
                new_warnings.append('Metapackage "%s" should not have other dependencies besides a '
                                    'buildtool_depend on catkin and run_depends.' % self.name)

        for warning in new_warnings:
            if warnings is None:
                print('WARNING: ' + warning, file=sys.stderr)
            elif warning not in warnings:
                warnings.append(warning)

        if errors:
            raise InvalidPackage('\n'.join(errors))


class Dependency(object):
    __slots__ = ['name', 'version_lt', 'version_lte', 'version_eq', 'version_gte', 'version_gt']

    def __init__(self, name, **kwargs):
        for attr in self.__slots__:
            value = kwargs[attr] if attr in kwargs else None
            setattr(self, attr, value)
        self.name = name
        # verify that no unknown keywords are passed
        unknown = set(kwargs.keys()).difference(self.__slots__)
        if unknown:
            raise TypeError('Unknown properties: %s' % ', '.join(unknown))

    def __eq__(self, other):
        if not isinstance(other, Dependency):
            return False
        return all([getattr(self, attr) == getattr(other, attr) for attr in self.__slots__])

    def __str__(self):
        return self.name


class Export(object):
    __slots__ = ['tagname', 'attributes', 'content']

    def __init__(self, tagname, content=None):
        self.tagname = tagname
        self.attributes = {}
        self.content = content

    def __str__(self):
        txt = '<%s' % self.tagname
        for key in sorted(self.attributes.keys()):
            txt += ' %s="%s"' % (key, self.attributes[key])
        if self.content:
            txt += '>%s</%s>' % (self.content, self.tagname)
        else:
            txt += '/>'
        return txt


class Person(object):
    __slots__ = ['name', 'email']

    def __init__(self, name, email=None):
        self.name = name
        self.email = email

    def __str__(self):
        name = self.name
        if not isinstance(name, str):
            name = name.encode('utf-8')
        if self.email is not None:
            return '%s <%s>' % (name, self.email)
        else:
            return '%s' % name

    def validate(self):
        if self.email is None:
            return
        if not re.match('^[-a-zA-Z0-9_%+]+(\.[-a-zA-Z0-9_%+]+)*@[-a-zA-Z0-9%]+(\.[-a-zA-Z0-9%]+)*\.[a-zA-Z]{2,}$', self.email):
            raise InvalidPackage('Invalid email "%s" for person "%s"' % (self.email, self.name))


class Url(object):
    __slots__ = ['url', 'type']

    def __init__(self, url, type_=None):
        self.url = url
        self.type = type_

    def __str__(self):
        return self.url


def parse_package_for_distutils(path=None):
    print('WARNING: %s/setup.py: catkin_pkg.package.parse_package_for_distutils() is deprecated. Please use catkin_pkg.python_setup.generate_distutils_setup(**kwargs) instead.' % os.path.basename(os.path.abspath('.')))
    from .python_setup import generate_distutils_setup
    data = {}
    if path is not None:
        data['package_xml_path'] = path
    return generate_distutils_setup(**data)


class InvalidPackage(Exception):
    pass


def package_exists_at(path):
    """
    Checks that a package exists at the given path

    :param path: path to a package
    :type path: str
    :returns: True if package exists in given path, else False
    :rtype: bool
    """
    return os.path.isdir(path) and os.path.isfile(os.path.join(path, PACKAGE_MANIFEST_FILENAME))


def parse_package(path, warnings=None):
    """
    Parse package manifest.

    :param path: The path of the package.xml file, it may or may not
        include the filename
    :param warnings: Print warnings if None or return them in the given list

    :returns: return :class:`Package` instance, populated with parsed fields
    :raises: :exc:`InvalidPackage`
    :raises: :exc:`IOError`
    """
    if os.path.isfile(path):
        filename = path
    elif package_exists_at(path):
        filename = os.path.join(path, PACKAGE_MANIFEST_FILENAME)
        if not os.path.isfile(filename):
            raise IOError('Directory "%s" does not contain a "%s"' % (path, PACKAGE_MANIFEST_FILENAME))
    else:
        raise IOError('Path "%s" is neither a directory containing a "%s" file nor a file' % (path, PACKAGE_MANIFEST_FILENAME))

    with open(filename, 'r') as f:
        try:
            return parse_package_string(f.read(), filename, warnings=warnings)
        except InvalidPackage as e:
            e.args = ['Invalid package manifest "%s": %s' % (filename, e.message)]
            raise


def parse_package_string(data, filename=None, warnings=None):
    """
    Parse package.xml string contents.

    :param data: package.xml contents, ``str``
    :param filename: full file path for debugging, ``str``
    :param warnings: Print warnings if None or return them in the given list
    :returns: return parsed :class:`Package`
    :raises: :exc:`InvalidPackage`
    """
    try:
        root = dom.parseString(data)
    except Exception as ex:
        raise InvalidPackage('The manifest contains invalid XML:\n%s' % ex)

    pkg = Package(filename)

    # verify unique root node
    nodes = _get_nodes(root, 'package')
    if len(nodes) != 1:
        raise InvalidPackage('The manifest must contain a single "package" root tag')
    root = nodes[0]

    # format attribute
    value = _get_node_attr(root, 'format', default=1)
    pkg.package_format = int(value)
    assert pkg.package_format in [1, 2], "Unable to handle package.xml format version '%d', please update catkin_pkg (e.g. on Ubuntu/Debian use: sudo apt-get update && sudo apt-get install --only-upgrade python-catkin-pkg)" % pkg.package_format

    # name
    pkg.name = _get_node_value(_get_node(root, 'name'))

    # version and optional abi
    version_node = _get_node(root, 'version')
    pkg.version = _get_node_value(version_node)
    pkg.version_abi = _get_node_attr(version_node, 'abi', default=None)

    # description
    pkg.description = _get_node_value(_get_node(root, 'description'), allow_xml=True, apply_str=False)

    # at least one maintainer, all must have email
    maintainers = _get_nodes(root, 'maintainer')
    for node in maintainers:
        pkg.maintainers.append(Person(
            _get_node_value(node, apply_str=False),
            _get_node_attr(node, 'email')
        ))

    # urls with optional type
    urls = _get_nodes(root, 'url')
    for node in urls:
        pkg.urls.append(Url(
            _get_node_value(node),
            _get_node_attr(node, 'type', default='website')
        ))

    # authors with optional email
    authors = _get_nodes(root, 'author')
    for node in authors:
        pkg.authors.append(Person(
            _get_node_value(node, apply_str=False),
            _get_node_attr(node, 'email', default=None)
        ))

    # at least one license
    licenses = _get_nodes(root, 'license')
    for node in licenses:
        pkg.licenses.append(_get_node_value(node))

    errors = []
    # dependencies and relationships
    pkg.build_depends = _get_dependencies(root, 'build_depend')
    pkg.buildtool_depends = _get_dependencies(root, 'buildtool_depend')
    if pkg.package_format == 1:
        run_depends = _get_dependencies(root, 'run_depend')
        for d in run_depends:
            pkg.build_export_depends.append(deepcopy(d))
            pkg.exec_depends.append(deepcopy(d))
    if pkg.package_format == 2:
        pkg.build_export_depends = _get_dependencies(root, 'build_export_depend')
        pkg.buildtool_export_depends = _get_dependencies(root, 'buildtool_export_depend')
        pkg.exec_depends = _get_dependencies(root, 'exec_depend')
        depends = _get_dependencies(root, 'depend')
        for dep in depends:
            # check for collisions with specific dependencies
            same_build_depends = ['build_depend' for d in pkg.build_depends if d.name == dep.name]
            same_build_export_depends = ['build_export_depend' for d in pkg.build_export_depends if d.name == dep.name]
            same_exec_depends = ['exec_depend' for d in pkg.exec_depends if d.name == dep.name]
            if same_build_depends or same_build_export_depends or same_exec_depends:
                errors.append("The generic dependency on '%s' is redundant with: %s" % (dep.name, ', '.join(same_build_depends + same_build_export_depends + same_exec_depends)))
            # only append non-duplicates
            if not same_build_depends:
                pkg.build_depends.append(deepcopy(dep))
            if not same_build_export_depends:
                pkg.build_export_depends.append(deepcopy(dep))
            if not same_exec_depends:
                pkg.exec_depends.append(deepcopy(dep))
        pkg.doc_depends = _get_dependencies(root, 'doc_depend')
    pkg.test_depends = _get_dependencies(root, 'test_depend')
    pkg.conflicts = _get_dependencies(root, 'conflict')
    pkg.replaces = _get_dependencies(root, 'replace')

    if pkg.package_format == 1:
        for test_depend in pkg.test_depends:
            same_build_depends = ['build_depend' for d in pkg.build_depends if d.name == test_depend.name]
            same_run_depends = ['run_depend' for d in pkg.run_depends if d.name == test_depend.name]
            if same_build_depends or same_run_depends:
                errors.append('The test dependency on "%s" is redundant with: %s' % (test_depend.name, ', '.join(same_build_depends + same_run_depends)))

    # exports
    export_node = _get_optional_node(root, 'export')
    if export_node is not None:
        exports = []
        for node in [n for n in export_node.childNodes if n.nodeType == n.ELEMENT_NODE]:
            export = Export(str(node.tagName), _get_node_value(node, allow_xml=True))
            for key, value in node.attributes.items():
                export.attributes[str(key)] = str(value)
            exports.append(export)
        pkg.exports = exports

    # verify that no unsupported tags and attributes are present
    unknown_root_attributes = [attr for attr in root.attributes.keys() if str(attr) != 'format']
    if unknown_root_attributes:
        errors.append('The "package" tag must not have the following attributes: %s' % ', '.join(unknown_root_attributes))
    depend_attributes = ['version_lt', 'version_lte', 'version_eq', 'version_gte', 'version_gt']
    known = {
        'name': [],
        'version': ['abi'],
        'description': [],
        'maintainer': ['email'],
        'license': [],
        'url': ['type'],
        'author': ['email'],
        'build_depend': depend_attributes,
        'buildtool_depend': depend_attributes,
        'test_depend': depend_attributes,
        'conflict': depend_attributes,
        'replace': depend_attributes,
        'export': [],
    }
    if pkg.package_format == 1:
        known.update({
            'run_depend': depend_attributes,
        })
    if pkg.package_format == 2:
        known.update({
            'build_export_depend': depend_attributes,
            'buildtool_export_depend': depend_attributes,
            'depend': depend_attributes,
            'exec_depend': depend_attributes,
            'doc_depend': depend_attributes,
        })
    nodes = [n for n in root.childNodes if n.nodeType == n.ELEMENT_NODE]
    unknown_tags = set([n.tagName for n in nodes if n.tagName not in known.keys()])
    if unknown_tags:
        errors.append('The manifest (with format version %d) must not contain the following tags: %s' % (pkg.package_format, ', '.join(unknown_tags)))
    for node in [n for n in nodes if n.tagName in known.keys()]:
        unknown_attrs = [str(attr) for attr in node.attributes.keys() if str(attr) not in known[node.tagName]]
        if unknown_attrs:
            errors.append('The "%s" tag must not have the following attributes: %s' % (node.tagName, ', '.join(unknown_attrs)))
        if node.tagName not in ['description', 'export']:
            subnodes = [n for n in node.childNodes if n.nodeType == n.ELEMENT_NODE]
            if subnodes:
                errors.append('The "%s" tag must not contain the following children: %s' % (node.tagName, ', '.join([n.tagName for n in subnodes])))

    if errors:
        raise InvalidPackage('Error(s) in %s:%s' % (filename, ''.join(['\n- %s' % e for e in errors])))

    pkg.validate(warnings=warnings)

    return pkg


def _get_nodes(parent, tagname):
    return [n for n in parent.childNodes if n.nodeType == n.ELEMENT_NODE and n.tagName == tagname]


def _get_node(parent, tagname):
    nodes = _get_nodes(parent, tagname)
    if len(nodes) != 1:
        raise InvalidPackage('The manifest must contain exactly one "%s" tags' % tagname)
    return nodes[0]


def _get_optional_node(parent, tagname):
    nodes = _get_nodes(parent, tagname)
    if len(nodes) > 1:
        raise InvalidPackage('The manifest must not contain more than one "%s" tags' % tagname)
    return nodes[0] if nodes else None


def _get_node_value(node, allow_xml=False, apply_str=True):
    if allow_xml:
        value = (''.join([n.toxml() for n in node.childNodes])).strip(' \n\r\t')
    else:
        value = (''.join([n.data for n in node.childNodes if n.nodeType == n.TEXT_NODE])).strip(' \n\r\t')
    if apply_str:
        value = str(value)
    return value


def _get_optional_node_value(parent, tagname, default=None):
    node = _get_optional_node(parent, tagname)
    if node is None:
        return default
    return _get_node_value(node)


def _get_node_attr(node, attr, default=False):
    """
    :param default: False means value is required
    """
    if node.hasAttribute(attr):
        return str(node.getAttribute(attr))
    if default is False:
        raise InvalidPackage('The "%s" tag must have the attribute "%s"' % (node.tagName, attr))
    return default


def _get_dependencies(parent, tagname):
    depends = []
    for node in _get_nodes(parent, tagname):
        depend = Dependency(_get_node_value(node))
        for attr in ['version_lt', 'version_lte', 'version_eq', 'version_gte', 'version_gt']:
            setattr(depend, attr, _get_node_attr(node, attr, None))
        depends.append(depend)
    return depends
