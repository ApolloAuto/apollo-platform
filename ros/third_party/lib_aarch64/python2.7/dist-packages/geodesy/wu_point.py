# Software License Agreement (BSD License)
#
# Copyright (C) 2012, Jack O'Quin
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
#  * Neither the name of the author nor of other contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
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
.. module:: wu_point

Convenience classes for manipulating way points and their associated
Universal Transverse Mercator (UTM_) coordinates.

.. _UTM: http://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system
.. _UUID: http://en.wikipedia.org/wiki/Uuid

.. _`geographic_msgs/GeographicMap`: http://ros.org/doc/api/geographic_msgs/html/msg/GeographicMap.html
.. _`geographic_msgs/GeoPoint`: http://ros.org/doc/api/geographic_msgs/html/msg/GeoPoint.html
.. _`geographic_msgs/RouteNetwork`: http://ros.org/doc/api/geographic_msgs/html/msg/RouteNetwork.html
.. _`geographic_msgs/WayPoint`: http://ros.org/doc/api/geographic_msgs/html/msg/WayPoint.html
.. _`geometry_msgs/Point`: http://ros.org/doc/api/geometry_msgs/html/msg/Point.html

"""

import math
import geodesy.utm

from geographic_msgs.msg import WayPoint
from geometry_msgs.msg import Point

class WuPoint():
    """
    :class:`WuPoint` represents a map way point with associated UTM_
    information.

    :param waypt: `geographic_msgs/WayPoint`_ message.
    :param utm: Corresponding :class:`geodesy.utm.UTMPoint` object. If None
                provided, the *utm* object will be created.
 
    .. describe:: str(wu_point)
 
       :returns: String representation of :class:`WuPoint` object.
    """

    def __init__(self, waypt, utm=None):
        """Constructor.

        Collects relevant information from the way point message, and
        creates the corresponding  :class:`geodesy.utm.UTMPoint`.
        """
        self.way_pt = waypt

        if utm:
            self.utm = utm
        else:
            # convert latitude and longitude to UTM (ignoring altitude)
            self.utm = geodesy.utm.fromMsg(waypt.position)

    def __str__(self):
        """:returns: String representation of :class:`WuPoint` """
        # uses python3-compatible str.format() method:
        return str(self.way_pt) + '\n' + str(self.utm)

    def is2D(self):
        """:returns: True if no altitude provided. """
        geo_point = self.way_pt.position
        return geo_point.altitude != geo_point.altitude

    def position(self):
        """:returns: Corresponding `geographic_msgs/GeoPoint`_ message."""
        return self.way_pt.position

    def toPoint(self):
        """:returns: Corresponding `geometry_msgs/Point`_ message."""
        return self.utm.toPoint()

    def toPointXY(self):
        """:returns: `geometry_msgs/Point`_ with X and Y coordinates, and Z coordinate of zero. """
        return Point(x = self.utm.easting, y = self.utm.northing)

    def toWayPoint(self):
        """:returns: Corresponding `geographic_msgs/WayPoint`_ message. """
        return self.way_pt

    def uuid(self):
        """:returns: UUID_ of way point. """
        return self.way_pt.id.uuid

class WuPointSet():
    """
    :class:`WuPointSet` is a container for the way points in a
    `geographic_msgs/GeographicMap`_ or
    `geographic_msgs/RouteNetwork`_ message.  UTM_ coordinates are
    available for each way point, but they are evaluated lazily, only
    when needed.

    :param points: array of `geographic_msgs/WayPoint`_ messages

    :class:`WuPointSet` supports these standard container operations:

    .. describe:: len(wu_set)

       :returns: The number of points in the set.

    .. describe:: wu_set[uuid]
 
       :returns: The point with key *uuid*.  Raises a :exc:`KeyError`
                 if *uuid* is not in the set.
 
    .. describe:: uuid in wu_set
 
       :returns: ``True`` if *wu_set* has a key *uuid*, else ``False``.
 
    .. describe:: uuid not in wu_set
 
       Equivalent to ``not uuid in wu_set``.
 
    .. describe:: iter(wu_set)
 
       :returns: An iterator over the points in the set.

    These methods are also provided:

    """

    def __init__(self, points):
        """Constructor.

        Collects relevant way point information from the way point
        array, and provides convenient access to the data.
        """
        self.points = points

        # Initialize way point information.
        self.way_point_ids = {}         # points symbol table
        self.n_points = len(self.points)
        for wid in xrange(self.n_points):
            self.way_point_ids[self.points[wid].id.uuid] = wid

        # Create empty list of UTM points, corresponding to map points.
        # They will be evaluated lazily, when first needed.
        self.utm_points = [None for wid in xrange(self.n_points)]

    def __contains__(self, item):
        """ Point set membership. """
        return item in self.way_point_ids

    def __getitem__(self, key):
        """
        :param key: UUID_ of desired point.
        :returns: Named :class:`WuPoint`.
        :raises: :exc:`KeyError` if no such point
        """
        index = self.way_point_ids[key]
        return self._get_point_with_utm(index)

    def __iter__(self):
        """ Points iterator. """
        self.iter_index = 0
        return self

    def __len__(self):
        """Point set length."""
        return self.n_points

    def _get_point_with_utm(self, index):
        """Get way point with UTM coordinates.

        Creates the corresponding :class:`UTMPoint`, if necessary.

        :param index: Index of point in self.
        :returns: Corresponding :class:`WuPoint` object.
        """
        way_pt = self.points[index]
        utm_pt = self.utm_points[index]
        if utm_pt is not None:
            utm_pt = geodesy.utm.fromMsg(way_pt.position)
            self.utm_points[index] = utm_pt
        return WuPoint(way_pt, utm=utm_pt)

    def distance2D(self, idx1, idx2):
        """ Compute 2D Euclidean distance between points.

        :param idx1: Index of first point.
        :param idx2: Index of second point.

        :returns: Distance in meters within the UTM XY
                  plane. Altitudes are ignored.
        """
        p1 = self._get_point_with_utm(idx1)
        p2 = self._get_point_with_utm(idx2)
        dx = p2.utm.easting - p1.utm.easting
        dy = p2.utm.northing - p1.utm.northing
        return math.sqrt(dx*dx + dy*dy)

    def distance3D(self, idx1, idx2):
        """ Compute 3D Euclidean distance between points.

        :param idx1: Index of first point.
        :param idx2: Index of second point.

        :returns: Distance in meters between two UTM points, including
                  altitudes.
        """
        p1 = self._get_point_with_utm(idx1)
        p2 = self._get_point_with_utm(idx2)
        dx = p2.utm.easting - p1.utm.easting
        dy = p2.utm.northing - p1.utm.northing
        dz = p2.utm.altitude - p1.utm.altitude
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def get(self, key, default=None):
        """ Get point, if defined.

        :param key: UUID_ of desired point.
        :param default: value to return if no such point.
        :returns: Named :class:`WuPoint`, if successful; otherwise default.
        """
        index = self.way_point_ids.get(key)
        if index is not None:
            return self._get_point_with_utm(index)
        else:
            return default

    def index(self, key, default=None):
        """ Get index of point, if defined.

        :param key: UUID_ of desired point.
        :param default: value to return if no such point.
        :returns: Index of point, if successful; otherwise default.
                  Beware: the index may be 0, which evaluates False as
                  a predicate, use ``is not None`` to test for
                  presence.
        """
        return self.way_point_ids.get(key, default)

    def next(self):
        """ Next iteration point.

        :returns: Next :class:`WuPoint`.
        :raises: :exc:`StopIteration` when finished.
        """
        i = self.iter_index
        if i >= self.n_points:
            raise StopIteration
        self.iter_index = i + 1
        return self._get_point_with_utm(i)
