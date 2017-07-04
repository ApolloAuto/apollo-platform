#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Open Source Robotics Foundation, Inc.
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
#  * Neither the name of Open Source Robotics Foundation, Inc. nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
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

import unittest
import rospy
import dynamic_reconfigure.client


class TestSimpleDynamicReconfigureClient(unittest.TestCase):

    def testsimple(self):
        client = dynamic_reconfigure.client.Client("ref_server", timeout=5)
        config = client.get_configuration(timeout=5)
        self.assertEqual(0, config['int_'])
        self.assertEqual(0.0, config['double_'])
        self.assertEqual('foo', config['str_'])
        self.assertEqual(False, config['bool_'])
        self.assertEqual(1.0, config['double_no_minmax'])
        self.assertEqual(2.0, config['double_no_max'])
        self.assertEqual(-1.0, config['deep_var_double'])


        int_ = 7
        double_ = 0.75
        str_ = 'bar'
        bool_ = True
        double_no_max_ = 1e+300
        double_no_minmax_ = -1e+300

        client.update_configuration(
            {"int_": int_, "double_": double_, "str_": str_,
             "bool_": bool_,
             "double_no_max": double_no_max_, "double_no_minmax": double_no_minmax_}
        )

        rospy.sleep(1.0)

        config = client.get_configuration(timeout=5)

        self.assertEqual(int_, config['int_'])
        self.assertEqual(double_, config['double_'])
        self.assertEqual(str_, config['str_'])
        self.assertEqual(type(str_), type(config['str_']))
        self.assertEqual(bool_, config['bool_'])
        self.assertEqual(double_no_max_, config['double_no_max'])
        self.assertEqual(double_no_minmax_, config['double_no_minmax'])

    def testmultibytestring(self):
        client = dynamic_reconfigure.client.Client("ref_server", timeout=5)
        config = client.get_configuration(timeout=5)
        self.assertEqual('bar', config['mstr_'])

        # Kanji for konnichi wa (hello)
        str_ = u"今日は"

        client.update_configuration(
            {"mstr_": str_}
        )

        rospy.sleep(1.0)

        config = client.get_configuration(timeout=5)

        self.assertEqual(u"今日は", config['mstr_'])
        self.assertEqual(type(u"今日は"), type(config['mstr_']))
        self.assertEqual(u"今日は", rospy.get_param('/ref_server/mstr_'))

        # Hiragana for konnichi wa (hello)
        str_ = u"こんにちは"

        client.update_configuration(
            {"mstr_": str_}
        )

        rospy.sleep(1.0)

        config = client.get_configuration(timeout=5)

        self.assertEqual(u"こんにちは", config['mstr_'])
        self.assertEqual(type(u"こんにちは"), type(config['mstr_']))
        self.assertEqual(u"こんにちは", rospy.get_param('/ref_server/mstr_'))

if __name__ == "__main__":
    import rostest
    rospy.init_node('simple_python_client_test')
    rostest.rosrun('dynamic_reconfigure', 'test_simple_dynamic_reconfigure_client_python', TestSimpleDynamicReconfigureClient)
