#!/usr/bin/env python
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

import roslib; roslib.load_manifest('dynamic_reconfigure')
import rospy
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
import time

# This example assumes that testserver in dynamic_reconfigure is running
# with its default node name, and will show how to reconfigure it from
# python. 
# 
# Note that testserver often changes the parameters it is given, so the
# values you get back from it will often be different from the ones you
# requested. Look at test/testserver.cpp to understand the changes that are
# being made to the parameters.
#
# In one window do:
# rosrun dynamic_reconfigure testserver
#
# In another window do:
# rosrun dynamic_reconfigure testclient.py 

def print_config(config):
    for k, v in config.iteritems():
        print k, ":", v
    print

# The config_callback (introduced below) receives a dictionary containing
# the current configuration of the server each time the server's
# configuration changes.
def config_callback(config): 
    print "Got callback, configuration is: "
    print_config(config)

def new_config_callback(config):
    global old_callback
    print "New callback is calling old callback..."
    old_callback(config)
    print "New callback is done..."
    print

# First you need to connect to the server. You can optionally specify a
# timeout and a config_callback that is called each time the server's
# configuration changes. If you do not indicate a timeout, the client is
# willing to wait forever for the server to be available.
#
# Note that the config_callback could get called before the constructor 
# returns.
rospy.init_node('testclient_py', anonymous=True)
client = DynamicReconfigureClient('/dynamic_reconfigure_test_server', config_callback=config_callback, timeout=5)
time.sleep(1)

# You can also get the configuration manually by calling get_configuration.
print "Configuration from get_configuration:"
print_config(client.get_configuration(timeout=5))
time.sleep(1)

# You can push changes to the server using the update_configuration method.
# You can set any subset of the node's parameters using this method. It
# returns out the full new configuration of the server (which may differ
# from what you requested if you asked for something illegal).
print "Configuration after setting int_ to 4:"
print_config(client.update_configuration({'int_' : 4}))
time.sleep(1)

print "Configuration after setting int_ to 0 and bool_ to True:"
print_config(client.update_configuration({'int_' : 0, 'bool_' : True}))
time.sleep(1)

# You can access constants defined in Test.cfg file in the following way:
import dynamic_reconfigure.cfg.TestConfig as Config
print "Medium is a constant that is set to 1:", Config.Test_Medium

# This is useful for setting enums:
print "Configuration after setting int_enum_ to Medium:"
print_config(client.update_configuration({'int_enum_' : Config.Test_Medium}))
time.sleep(1)

# You can use the get_config_callback and set_config_callback methods to
# get the current callback or change the callback. It is sometimes useful
# not to set the callback in the constructor, but to wait until more
# initialization has been done first. Again, the callback will be called
# immediately if a configuration is available.
old_callback = client.get_config_callback()
client.set_config_callback(new_config_callback)

time.sleep(1)

# When you are done, you can close the client.
client.close()
