#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
#  * Neither the name of the Willow Garage nor the names of its
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

import rosunit
import subprocess
import unittest
import os
from camera_calibration_parsers import readCalibration

class TestParser(unittest.TestCase):
    def test_ini(self):
        for files in [('calib5.ini', 'calib5.yaml'), ('calib8.ini', 'calib8.yaml')]:
            for dir in [ '', './']:
                p = subprocess.Popen('rosrun camera_calibration_parsers convert $(rospack find camera_calibration_parsers)/test/%s %s%s' % (files[0], dir, files[1]), shell=True, stderr=subprocess.PIPE)
                out, err = p.communicate()
                self.assertEqual(err, '')

    def test_readCalibration(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        camera_name, camera_info = readCalibration(os.path.join(script_dir, 'calib5.ini'))
        self.assertEqual(camera_name, 'mono_left')
        self.assertEqual(camera_info.height, 480)
        self.assertEqual(camera_info.width, 640)
        self.assertEqual(camera_info.P[0], 262.927429)
        
        camera_name, camera_info = readCalibration(os.path.join(script_dir, 'calib8.ini'))
        self.assertEqual(camera_info.distortion_model, 'rational_polynomial')

if __name__ == '__main__':
    rosunit.unitrun('camera_calibration_parsers', 'parser', TestParser)
