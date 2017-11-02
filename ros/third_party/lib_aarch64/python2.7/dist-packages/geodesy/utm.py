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
.. module:: utm

Universal Transverse Mercator coordinate module.

:todo: add Universal Polar Stereographic (UPS_) support

.. _MGRS: http://en.wikipedia.org/wiki/Military_grid_reference_system
.. _`PROJ.4`: http://trac.osgeo.org/proj/
.. _pyproj: http://pypi.python.org/pypi/pyproj 
.. _UPS: http://en.wikipedia.org/wiki/Universal_Polar_Stereographic_coordinate_system
.. _UTM: http://en.wikipedia.org/wiki/Universal_Transverse_Mercator_coordinate_system

.. _`geometry_msgs/Point`: http://ros.org/doc/api/geometry_msgs/html/msg/Point.html
.. _`geographic_msgs/GeoPoint`: http://ros.org/doc/api/geographic_msgs/html/msg/GeoPoint.html

"""

import math
import pyproj

from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg   import Point

class UTMPoint:
    """Universal Transverse Mercator (UTM_) point class.

    For outdoor robotics applications, Euclidean projections like UTM_
    are easier to work with than latitude and longitude.  This system
    is slightly more general than strict UTM.  It is based on the
    Military Grid Reference System (MGRS_), which can be extended to
    cover the poles, allowing well-defined transformations for every
    latitude and longitude.

    This implementation uses the pyproj_ wrapper for the `PROJ.4`_
    Cartographic Projections Library.

    :param easting: UTM easting (meters)
    :param northing: UTM northing (meters)
    :param altitude: altitude above the WGS84 ellipsoid (meters),
                     none if NaN.
    :param zone: UTM longitude zone
    :param band: MGRS latitude band letter

    """

    def __init__(self, easting=float('nan'), northing=float('nan'),
                 altitude=float('nan'), zone=0, band=' '):
        """Construct UTMPoint object. """
        self.easting = easting
        self.northing = northing
        self.altitude = altitude
        self.zone = zone
        self.band = band

    def __str__(self):
        """ :returns: string representation of :class:`UTMPoint`. """
        # uses python3-compatible str.format() method:
        return 'UTM: [{0:.3f}, {1:.3f}, {2:.3f}, {3}{4}]'.format(
            self.easting, self.northing, self.altitude, self.zone, self.band)

    def gridZone(self):
        """:returns: (zone, band) tuple. """
        return (self.zone, self.band)

    def is2D(self):
        """:returns: True if altitude is not defined."""
        return self.altitude != self.altitude

    def toPoint(self):
        """:returns: corresponding `geometry_msgs/Point`_ message.
        :todo: clamp message longitude to [-180..180]
        """
        if not self.valid():
            raise ValueError('invalid UTM point: ' + str(self))
        pt = Point(x = self.easting, y = self.northing)
        if not self.is2D():
            pt.z = self.altitude
        return pt

    def toMsg(self):
        """:returns: corresponding `geographic_msgs/GeoPoint`_ message.
        :todo: clamp message longitude to [-180..180]
        """
        if not self.valid():
            raise ValueError('invalid UTM point: ' + str(self))
        utm_proj = pyproj.Proj(proj='utm', zone=self.zone, datum='WGS84')
        msg = GeoPoint(altitude=self.altitude)
        msg.longitude, msg.latitude = utm_proj(self.easting, self.northing,
                                               inverse=True)
        return msg

    def valid(self):
        """:returns: True if this is a valid UTM point. """
        return (self.easting == self.easting
                and self.northing == self.northing
                and self.band != ' ')

def fromLatLong(latitude, longitude, altitude=float('nan')):
    """Generate :class:`UTMPoint` from latitude, longitude and (optional) altitude.

    Latitude and longitude are expressed in degrees, relative to the
    WGS84 ellipsoid.

    :param latitude: [degrees], negative is South.
    :param longitude: [degrees], negative is West.
    :param altitude: [meters], negative is below the ellipsoid.

    :returns: :class:`UTMPoint` object.
    """
    z, b = gridZone(latitude, longitude)
    utm_proj = pyproj.Proj(proj='utm', zone=z, datum='WGS84')
    e, n = utm_proj(longitude, latitude)
    return UTMPoint(easting=e, northing=n, altitude=altitude, zone=z, band=b)

def fromMsg(msg):
    """
    :param msg: `geographic_msgs/GeoPoint`_ message.
    :returns: :class:`UTMPoint` object.
    """
    return fromLatLong(msg.latitude, msg.longitude, msg.altitude)

def gridZone(lat, lon):
    """Find UTM zone and MGRS band for latitude and longitude.

       :param lat: latitude in degrees, negative is South.
       :param lon: longitude in degrees, negative is West.
       :returns: (zone, band) tuple.
       :raises: :exc:`ValueError` if lon not in [-180..180] or if lat
                has no corresponding band letter.

       :todo: handle polar (UPS_) zones: A, B, Y, Z.
    """
    if -180.0 > lon or lon > 180.0:
        raise ValueError('invalid longitude: ' + str(lon))
    zone = int((lon + 180.0)//6.0) + 1
    band = ' '
    if    84 >= lat and lat >= 72: band = 'X'
    elif  72 > lat and lat >= 64:  band = 'W'
    elif  64 > lat and lat >= 56:  band = 'V'
    elif  56 > lat and lat >= 48:  band = 'U'
    elif  48 > lat and lat >= 40:  band = 'T'
    elif  40 > lat and lat >= 32:  band = 'S'
    elif  32 > lat and lat >= 24:  band = 'R'
    elif  24 > lat and lat >= 16:  band = 'Q'
    elif  16 > lat and lat >= 8:   band = 'P'
    elif   8 > lat and lat >= 0:   band = 'N'
    elif   0 > lat and lat >= -8:  band = 'M'
    elif  -8 > lat and lat >= -16: band = 'L'
    elif -16 > lat and lat >= -24: band = 'K'
    elif -24 > lat and lat >= -32: band = 'J'
    elif -32 > lat and lat >= -40: band = 'H'
    elif -40 > lat and lat >= -48: band = 'G'
    elif -48 > lat and lat >= -56: band = 'F'
    elif -56 > lat and lat >= -64: band = 'E'
    elif -64 > lat and lat >= -72: band = 'D'
    elif -72 > lat and lat >= -80: band = 'C'
    else: raise ValueError('latitude out of UTM range: ' + str(lat))
    return (zone, band)
