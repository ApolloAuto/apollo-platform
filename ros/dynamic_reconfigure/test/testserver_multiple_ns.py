#! /usr/bin/env python
#*********************************************************************
# Software License Agreement (BSD License)
#
#  Copyright (c) 2009-2010, Willow Garage, Inc.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#********************************************************************/

import roslib; roslib.load_manifest('dynamic_reconfigure')
import rospy
import dynamic_reconfigure.server
from dynamic_reconfigure.cfg import TestConfig
import time

def main():
    rospy.init_node("python_test_multiple_ns_server")
    dynamic_reconfigure.server.Server(TestConfig, reconfigure)
    dynamic_reconfigure.server.Server(TestConfig, reconfigure_alternate_ns,
                                      "~alternate_ns")
    dynamic_reconfigure.server.Server(TestConfig, reconfigure_2lvls_ns,
                                      "~alternate_ns/second_lvl")
    dynamic_reconfigure.server.Server(TestConfig, reconfigure_absolute_ns,
                                      "/absolute_ns")
    while not rospy.is_shutdown():
        time.sleep(0.1)

def reconfigure(config, level):
    print config

    config['int_'] |= 1;
    config['double_'] = -config['double_'];
    config['str_'] += "A";
    config['bool_'] = not config['bool_'];
    config['level'] = level;

    rospy.loginfo("Reconfigured to     : %i %f %s %s %i"%(config['int_'], config['double_'], config['str_'], config['bool_'], config['level']))

    return config # Returns the updated configuration.

def reconfigure_default(config,level):
    rospy.loginfo("Reconfigure request on default ns")
    return reconfigure(config, level)

def reconfigure_alternate_ns(config,level):
    rospy.loginfo("Reconfigure request on alternate_ns")
    return reconfigure(config, level)

def reconfigure_2lvls_ns(config,level):
    rospy.loginfo("Reconfigure request on alternate_ns/second_lvl")
    return reconfigure(config, level)

def reconfigure_absolute_ns(config,level):
    rospy.loginfo("Reconfigure request on /absolute_ns")
    return reconfigure(config, level)

if __name__ == '__main__':
    main()
