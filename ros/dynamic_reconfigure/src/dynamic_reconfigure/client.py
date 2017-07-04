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
import sys
import threading
import time
import types
from dynamic_reconfigure import DynamicReconfigureParameterException
from dynamic_reconfigure.srv import Reconfigure as ReconfigureSrv
from dynamic_reconfigure.msg import Config as ConfigMsg
from dynamic_reconfigure.msg import ConfigDescription as ConfigDescrMsg
from dynamic_reconfigure.msg import IntParameter, BoolParameter, StrParameter, DoubleParameter, ParamDescription
from dynamic_reconfigure.encoding import *

class Client(object):
    """
    Python dynamic_reconfigure client API
    """
    def __init__(self, name, timeout=None, config_callback=None, description_callback=None):
        """
        Connect to dynamic_reconfigure server and return a client object
        
        @param name: name of the server to connect to (usually the node name)
        @type  name: str
        @param timeout: time to wait before giving up
        @type  timeout: float
        @param config_callback: callback for server parameter changes
        @param description_callback: internal use only as the API has not stabilized
        """
        self.name              = name
        self.config            = None
        self.param_description = None
        self.group_description = None
        
        self._param_types = None

        self._cv = threading.Condition()

        self._config_callback      = config_callback
        self._description_callback = description_callback

        self._set_service      = self._get_service_proxy('set_parameters', timeout)       
        self._descriptions_sub = self._get_subscriber('parameter_descriptions', ConfigDescrMsg, self._descriptions_msg)
        self._updates_sub      = self._get_subscriber('parameter_updates',      ConfigMsg,      self._updates_msg)

    def get_configuration(self, timeout=None):
        """
        Return the latest received server configuration (wait to receive
        one if none have been received)

        @param timeout: time to wait before giving up
        @type  timeout: float
        @return: dictionary mapping parameter names to values or None if unable to retrieve config.
        @rtype: {str: value}
        """
        if timeout is None or timeout == 0.0:
            if self.get_configuration(timeout=1.0) is None:
                print >> sys.stderr, 'Waiting for configuration...'
                
                with self._cv:
                    while self.config is None:
                        if rospy.is_shutdown():
                            return None
                        self._cv.wait()
        else:
            start_time = time.time()
            with self._cv:
                while self.config is None:
                    if rospy.is_shutdown():
                        return None
                    secs_left = timeout - (time.time() - start_time)
                    if secs_left <= 0.0:
                        break
                    self._cv.wait(secs_left)

        return self.config

    def get_parameter_descriptions(self, timeout=None):
        """
        UNSTABLE. Return a description of the parameters for the server.
        Do not use this method as the type that is returned may change.
        
        @param timeout: time to wait before giving up
        @type  timeout: float
        """
        if timeout is None or timeout == 0.0:
            with self._cv:
                while self.param_description is None:
                    if rospy.is_shutdown():
                        return None
                    self._cv.wait()
        else:
            start_time = time.time()
            with self._cv:
                while self.param_description is None:
                    if rospy.is_shutdown():
                        return None
                    secs_left = timeout - (time.time() - start_time)
                    if secs_left <= 0.0:
                        break
                    self._cv.wait(secs_left)

        return self.param_description

    def get_group_descriptions(self, timeout=None):
        if timeout is None or timeout == 0.0:
            with self._cv:
                while self.group_description is None:
                    if rospy.is_shutdown():
                        return None
                    self._cv.wait()
        else:
            start_time = time.time()
            with self._cv:
                while self.group_description is None:
                    if rospy.is_shutdown():
                        return None
                    secs_left = timeout - (time.time() - start_time)
                    if secs_left <= 0.0:
                        break
                    self._cv.wait(secs_left)

        return self.group_description

    def update_configuration(self, changes):
        """
        Change the server's configuration

        @param changes: dictionary of key value pairs for the parameters that are changing
        @type  changes: {str: value}
        """
        # Retrieve the parameter descriptions
        if self.param_description is None:
            self.get_parameter_descriptions()

        # Cast the parameters to the appropriate types
        if self.param_description is not None:
            for name, value in list(changes.items())[:]:
                if name != 'groups':
                    dest_type = self._param_types.get(name)
                    if dest_type is None:
                        raise DynamicReconfigureParameterException('don\'t know parameter: %s' % name)
                
                    try:
                        found = False
                        descr = [x for x in self.param_description if x['name'].lower() == name.lower()][0]

                        # Fix not converting bools properly
                        if dest_type is bool and type(value) is str:
                            changes[name] = value.lower() in ("yes", "true", "t", "1")
                            found = True
                        # Handle enums
                        elif type(value) is str and not descr['edit_method'] == '':
                            enum_descr = eval(descr['edit_method'])
                            found = False
                            for const in enum_descr['enum']:
                                if value.lower() == const['name'].lower():
                                    val_type = self._param_type_from_string(const['type'])
                                    changes[name] = val_type(const['value'])
                                    found = True
                        if not found:
                            if sys.version_info.major < 3:
                                if type(value) is unicode:
                                    changes[name] = unicode(value)
                                else:
                                    changes[name] = dest_type(value)
                            else:
                                changes[name] = dest_type(value)

                    except ValueError as e:
                        raise DynamicReconfigureParameterException('can\'t set parameter \'%s\' of %s: %s' % (name, str(dest_type), e))

        if 'groups' in changes.keys():
            changes['groups'] = self.update_groups(changes['groups'])

        config = encode_config(changes)
        msg    = self._set_service(config).config
        if self.group_description is None:
            self.get_group_descriptions()
        resp   = decode_config(msg, self.group_description)

        return resp

    def update_groups(self, changes):
        """
        Changes the servers group configuration

        @param changes: dictionary of key value pairs for the parameters that are changing
        @type  changes: {str: value}
        """
        
        descr = self.get_group_descriptions()

        groups = []
        def update_state(group, description):
            for p,g in description['groups'].items():
                if g['name'] == group:
                    description['groups'][p]['state'] = changes[group]
                else:
                    update_state(group, g)
            return description
 
        for change in changes:
            descr = update_state(change, descr)

        return descr

    def close(self):
        """
        Close connections to the server
        """
        self._descriptions_sub.unregister()
        self._updates_sub.unregister()

    ## config_callback

    def get_config_callback(self):
        """
        Retrieve the config_callback
        """
        return self._config_callback

    def set_config_callback(self, value):
        """
        Set the config_callback
        """
        self._config_callback = value
        if self._config_callback is not None:
            self._config_callback(self.config)

    config_callback = property(get_config_callback, set_config_callback)

    ## description_callback        

    def get_description_callback(self):
        """
        Get the current description_callback
        """
        return self._description_callback

    def set_description_callback(self, value):
        """
        UNSTABLE. Set the description callback. Do not use as the type of the
        description callback may change.
        """
        self._description_callback = value
        if self._description_callback is not None:
            self._description_callback(self.param_description)

    description_callback = property(get_description_callback, set_description_callback)

    # Implementation

    def _get_service_proxy(self, suffix, timeout):
        service_name = rospy.resolve_name(self.name + '/' + suffix)
        if timeout is None or timeout == 0.0:
            try:
                rospy.wait_for_service(service_name, 1.0)
            except rospy.exceptions.ROSException:
                print >> sys.stderr, 'Waiting for service %s...' % service_name
                rospy.wait_for_service(service_name, timeout)
        else:
            rospy.wait_for_service(service_name, timeout)

        return rospy.ServiceProxy(service_name, ReconfigureSrv)

    def _get_subscriber(self, suffix, type, callback):
        topic_name = rospy.resolve_name(self.name + '/' + suffix)
        
        return rospy.Subscriber(topic_name, type, callback=callback)

    def _updates_msg(self, msg):
        if self.group_description is None:
            self.get_group_descriptions()
        self.config = decode_config(msg, self.group_description)
        
        with self._cv:
            self._cv.notifyAll()
        if self._config_callback is not None:
            self._config_callback(self.config)

    def _descriptions_msg(self, msg):
        self.group_description = decode_description(msg)
        self.param_description = extract_params(self.group_description)

        # Build map from parameter name to type
        self._param_types = {}
        for p in self.param_description:
            n, t = p.get('name'), p.get('type')
            if n is not None and t is not None:
                self._param_types[n] = self._param_type_from_string(t)

        with self._cv:
            self._cv.notifyAll()
        if self._description_callback is not None:
            self._description_callback(self.param_description)

    def _param_type_from_string(self, type_str):
        if   type_str == 'int':    return int
        elif type_str == 'double': return float
        elif type_str == 'str':    return str
        elif type_str == 'bool':   return bool
        else:
            raise DynamicReconfigureParameterException('parameter has unknown type: %s. This is a bug in dynamic_reconfigure.' % type_str)
