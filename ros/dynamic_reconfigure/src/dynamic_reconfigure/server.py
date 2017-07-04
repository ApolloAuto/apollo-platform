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

"""
Python client API for dynamic_reconfigure (L{DynamicReconfigureClient}) as well as 
example server implementation (L{DynamicReconfigureServer}).
"""

from __future__ import with_statement

try:
    import roslib; roslib.load_manifest('dynamic_reconfigure')
except:
    pass
import rospy
import rosservice                  
import threading
import time
import copy
from dynamic_reconfigure import DynamicReconfigureCallbackException
from dynamic_reconfigure.srv import Reconfigure as ReconfigureSrv
from dynamic_reconfigure.msg import Config as ConfigMsg
from dynamic_reconfigure.msg import ConfigDescription as ConfigDescrMsg
from dynamic_reconfigure.msg import IntParameter, BoolParameter, StrParameter, DoubleParameter, ParamDescription
from dynamic_reconfigure.encoding import *

class Server(object):
    def __init__(self, type, callback, namespace=""):
        self.mutex = threading.Lock()
        if not namespace:
            self.ns = "~"
        else:
            if namespace[0] not in ["/", "~"]:
                namespace = "~" + namespace
            self.ns = (namespace + "/").replace("//", "/")

        self.type = type
        self.config = type.defaults.copy()

        self.description = encode_description(type)
        self._copy_from_parameter_server()
        self.callback = callback
        self._clamp(self.config) 

        # setup group defaults
        self.config['groups'] = get_tree(self.description)
        self.config = initial_config(encode_config(self.config), type.config_description)

        self.descr_topic = rospy.Publisher(self.ns + 'parameter_descriptions', ConfigDescrMsg, latch=True, queue_size=10)
        self.descr_topic.publish(self.description);

        self.update_topic = rospy.Publisher(self.ns + 'parameter_updates', ConfigMsg, latch=True, queue_size=10)
        self._change_config(self.config, ~0) # Consistent with the C++ API, the callback gets called with level=~0 (i.e. -1)

        self.set_service = rospy.Service(self.ns + 'set_parameters', ReconfigureSrv, self._set_callback)

    def update_configuration(self, changes):
        with self.mutex:
            new_config = copy.deepcopy(self.config)
            new_config.update(changes)
            self._clamp(new_config)
            return self._change_config(new_config, self._calc_level(new_config, self.config))

    def _copy_from_parameter_server(self):
        for param in extract_params(self.type.config_description):
            try:
                self.config[param['name']] = rospy.get_param(self.ns + param['name'])
            except KeyError:
                pass

    def _copy_to_parameter_server(self):
        for param in extract_params(self.type.config_description):
            rospy.set_param(self.ns + param['name'], self.config[param['name']])

    def _change_config(self, config, level):
        self.config = self.callback(config, level)
        if self.config is None:
            msg = 'Reconfigure callback should return a possibly updated configuration.'
            rospy.logerr(msg)
            raise DynamicReconfigureCallbackException(msg)
        
        self._copy_to_parameter_server()
        
        self.update_topic.publish(encode_config(self.config))

        return self.config
   
    def _calc_level(self, config1, config2):
        level = 0
        for param in extract_params(self.type.config_description):
            if config1[param['name']] != config2[param['name']]:
                level |= param['level']

        return level

    def _clamp(self, config):
        for param in extract_params(self.type.config_description): 
            maxval = self.type.max[param['name']] 
            minval = self.type.min[param['name']] 
            val = config[param['name']]
            if val > maxval and maxval != "": 
                config[param['name']] = maxval 
            elif val < minval and minval != "": 
                config[param['name']] = minval 

    def _set_callback(self, req):
        return encode_config(self.update_configuration(decode_config(req.config, self.type.config_description)))
