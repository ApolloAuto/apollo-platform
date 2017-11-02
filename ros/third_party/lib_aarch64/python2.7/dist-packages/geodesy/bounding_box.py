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
.. module:: bounding_box

Bounding box functions for geographic maps.

.. _`geographic_msgs/BoundingBox`: http://ros.org/doc/api/geographic_msgs/html/msg/BoundingBox.html

"""

from geographic_msgs.msg import BoundingBox

def getLatLong(bbox):
    """
    Get the tuple of minimum and maximum latitudes and longitudes.

    :param bbox: `geographic_msgs/BoundingBox`_.
    :returns: (min_lat, min_lon, max_lat, max_lon)
    """
    return (bbox.min_pt.latitude, bbox.min_pt.longitude,
            bbox.max_pt.latitude, bbox.max_pt.longitude)

def is2D(bbox):
    """
    Two-dimensional bounding box predicate.

    :param bbox: `geographic_msgs/BoundingBox`_.
    :returns: True if *bbox* matches any altitude.
    """
    return bbox.min_pt.altitude != bbox.min_pt.altitude

def isGlobal(bbox):
    """
    Global bounding box predicate.

    :param bbox: `geographic_msgs/BoundingBox`_.
    :returns: True if *bbox* matches any global coordinate.
    """
    return bbox.min_pt.latitude != bbox.min_pt.latitude

def makeBounds2D(min_lat, min_lon, max_lat, max_lon):
    """
    Create a 2D geographic bounding box (ignoring altitudes).

    :param min_lat: Minimum latitude.
    :param min_lon: Minimum longitude.
    :param max_lat: Maximum latitude.
    :param max_lon: Maximum longitude.
    :returns: `geographic_msgs/BoundingBox`_ object.
    """
    bbox = BoundingBox()
    bbox.min_pt.latitude =  min_lat
    bbox.min_pt.longitude = min_lon
    bbox.min_pt.altitude =  float('nan')
    bbox.max_pt.latitude =  max_lat
    bbox.max_pt.longitude = max_lon
    bbox.max_pt.altitude =  float('nan')
    return bbox

def makeBounds3D(min_lat, min_lon, min_alt,
                 max_lat, max_lon, max_alt):
    """
    Create a 3D geographic bounding box (including altitudes).

    :param min_lat: Minimum latitude.
    :param min_lon: Minimum longitude.
    :param min_alt: Minimum altitude.
    :param max_lat: Maximum latitude.
    :param max_lon: Maximum longitude.
    :param max_alt: Maximum altitude.
    :returns: `geographic_msgs/BoundingBox`_ object.
    """
    bbox = BoundingBox()
    bbox.min_pt.latitude =  min_lat
    bbox.min_pt.longitude = min_lon
    bbox.min_pt.altitude =  min_alt
    bbox.max_pt.latitude =  max_lat
    bbox.max_pt.longitude = max_lon
    bbox.max_pt.altitude =  max_alt
    return bbox

def makeGlobal():
    """
    Create a global bounding box, which matches any valid coordinate.

    :returns: `geographic_msgs/BoundingBox`_ object.
    """
    bbox = BoundingBox()
    bbox.min_pt.latitude = float('nan')
    bbox.min_pt.longitude = float('nan')
    bbox.min_pt.altitude = float('nan')
    bbox.max_pt.latitude = float('nan')
    bbox.max_pt.longitude = float('nan')
    bbox.max_pt.altitude = float('nan')
    return bbox
