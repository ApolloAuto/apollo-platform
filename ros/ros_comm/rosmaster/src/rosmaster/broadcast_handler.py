# Software License Agreement (BSD License)
#
# Copyright (c) 2017, The Apollo Authors.
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
#
# Revision $Id$

import os
import json
import time
import socket
import struct
import decimal
import logging
import traceback
import threading
import functools

import rospy.core
from rospy.core import signal_shutdown
from rospy.impl.registration import Registration
from rospy.impl.registration import get_topic_manager
from rospy.impl.registration import get_service_manager
from rospy.impl.registration import get_node_handler
from rosgraph.network import parse_http_host_and_port,get_host_name

import sys
env = os.environ.get('LD_LIBRARY_PATH')
for sub_path in env.split(':'):
    sys.path.append(sub_path)
from rospy.impl import participant

TIMESTAMP = 'timestamp'
NODE_NAME = 'node_name'
XMLRPC_URI = 'xmlrpc_uri'
REQUEST_TYPE = 'request_type'

NODE_TIME = "node_time"
TOPIC_NAME = "topic_name"
TOPIC_TYPE = "topic_type"
TOPIC_URI = "topic_uri"

SERVICE_NAME = "service_name"
SERVICE_TYPE = "service_type"
SERVICE_URI = "service_uri"

class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

def byteify(input):
    """
    Convert unicode to str.
    """
    if isinstance(input, dict):
        return {byteify(key): byteify(value) for key, value in input.iteritems()}
    elif isinstance(input, list):
        return [byteify(element) for element in input]
    elif isinstance(input, unicode):
        return input.encode('utf-8')
    else:
        return input

class BroadcastHandler(object):

    """
    BroadcastHandler.
    """
    __metaclass__ = Singleton

    def __init__(self, handler):
        """
        brief info for: __init__
        """
        super(BroadcastHandler, self).__init__()

        self._logger = logging.getLogger(__name__)
        self._logger.setLevel(logging.INFO)

        self.callback = ["registerPublisher",
                         "unregisterPublisher",
                         "registerSubscriber",
                         "unregisterSubscriber",
                         "registerService",
                         "unregisterService",
                         "lookupService",
                         "getTopicTypes",
                         "lookupNode",
                         ]

        self._handler = handler
        self._name = "rosmaster"
        self._participant = participant.Participant(self._name)
        self._participant.init_py()
        self._broardcast_manager_thread = threading.Thread(
            target=self.run, args=())
        self._broardcast_manager_thread.setDaemon(True)
        self._broardcast_manager_thread.start()

    def run(self):
        """
        brief info for: thread run method
        """
        self._logger.debug("starting broadcast_manager!")

        while True:
            try:
                msg = self._participant.read_msg()
                if msg is None:
                    continue
                if(len(msg) > 0):
                    data = self._unpack_msg(msg.strip())
                        
                    self._logger.debug("recv data: %s " % data)

                    try:
                        cb = '_' + data[REQUEST_TYPE] + "Callback"
                        func = getattr(self, cb)
                        func(data)
                    except AttributeError:
                        pass
                else:
                    time.sleep(0.005)
            except Exception as e:
                self._logger.error("broadcast_manager thread error is  %s" % e)
            finally:
                pass

    def getUri(self, caller_id):
        """
        getUri
        """        
        return 1, "", self._uri 

    def getPid(self, caller_id):
        """
        Get the PID of this server
        """
        return 1, "", os.getpid()

    def _registerPublisherCallback(self, data):
        name = data[NODE_NAME]
        topic = data[TOPIC_NAME]
        datatype = data[TOPIC_TYPE]
        uri = data[XMLRPC_URI]
        self._handler.registerPublisher(name, topic, datatype, uri)

    def _unregisterPublisherCallback(self, data):
        name = data[NODE_NAME]
        topic = data[TOPIC_NAME]
        uri = data[TOPIC_URI]
        self._handler.unregisterPublisher(name, topic, uri)

    def _registerSubscriberCallback(self, data):
        name = data[NODE_NAME]
        topic = data[TOPIC_NAME]
        datatype = data[TOPIC_TYPE]
        uri = data[XMLRPC_URI]
        self._handler.registerSubscriber(name, topic, datatype, uri)

    def _unregisterSubscriberCallback(self, data):
        name = data[NODE_NAME]
        topic = data[TOPIC_NAME]
        uri = data[TOPIC_URI]
        self._handler.unregisterSubscriber(name, topic, uri)

    def _registerServiceCallback(self, data):
        name = data[NODE_NAME]
        service_name = data[SERVICE_NAME]
        service_uri = data[SERVICE_URI]
        uri = data[XMLRPC_URI]
        self._handler.registerService(name, service_name, service_uri, uri)

    def _unregisterServiceCallback(self, data):
        name = data[NODE_NAME]
        service_name = data[SERVICE_NAME]
        service_uri = data[SERVICE_URI]
        self._handler.unregisterService(name, service_name, service_uri)
 
    def _send(self, data):
        """
        brief info for: Get _master_handler internal dict stuct according to dict_type
        """
        self._participant.send(data)

    def _recv(self, size=1024):
        """
        brief info for: Get _master_handler internal dict stuct according to dict_type
        """
        msg = addr = None
        try:
            msg, addr = self._sock.recvfrom(size)
        except Exception as e:
            self._logger.error("socket recv error is  %s" % e)
            self._logger.error(traceback.format_exc())
        finally:
            pass
        return msg, addr

    def _unpack_msg(self, msg):
        try:
            data = json.loads(msg, object_hook=byteify)
        except Exception as e:
            self._logger.error("parse json failed! %s" % e)
        return data

    def _pack_msg(self, data):
        return json.dumps(data)
