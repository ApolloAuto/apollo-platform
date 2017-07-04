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

NODE_NAME = 'node_name'
TIMESTAMP = 'timestamp'
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

class BroadcastManager(object):
    """
    BroadcastManager.
    """
    __metaclass__ = Singleton

    def __init__(self, name=None, callback=None):
        """
        brief info for: __init__
        """
        super(BroadcastManager, self).__init__()

        self._logger = logging.getLogger(__name__)
        self._logger.setLevel(logging.INFO)

        if callback is None:
            self.callback = ["registerPublisher",
                             "unregisterPublisher",
                             "registerSubscriber",
                             "unregisterSubscriber",
                             "registerService",
                             "unregisterService",
                             "lookupService",
                             "getTopicTypes",
                             "lookupNode",
                             "registerNode",
                             ]
        else:
            self.callback = callback

        self._pubs = {}
        self._subs = {}
        self._service_cache = {}
        self._node_info = []
        self._topic_info = []
        self._pub_history = {}

        self._node_time = str(int(round(time.time()*1000)))
        if get_node_handler() is not None:
            self._name = get_node_handler().name
            while get_node_handler().uri is None:
                time.sleep(0.01) 
            self._uri = get_node_handler().uri
        else:
            if name is None:
                self._name = "_null_name"
            else:
                self._name = name
            self._uri = "_null_uri"
      
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
        self._register_node()

        while True:
            try:
                msg = self._participant.read_msg()
                if msg is None:
                    continue
                if(len(msg) > 0):
                    data = self._unpack_msg(msg.strip())
                        
                    if self._name == data[NODE_NAME]:
                        if data[REQUEST_TYPE] == "registerNode" \
                                and self._uri != data[XMLRPC_URI] \
                                and int(data[NODE_TIME]) > int(self._node_time):
                            signal_shutdown("it has already same name node, exit it now.")
                        continue

                    self._logger.debug("recv data: %s " % data)

                    cb = '_' + data[REQUEST_TYPE] + "Callback"
                    try:
                        func = getattr(self, cb)
                        func(data)
                    except AttributeError:
                        pass
                else:
                    time.sleep(0.005)

            except Exception as e:
                self._logger.error("[broadcast_manager] Unexpected error:  %s" % e)
            finally:
                pass

    def updateHandler(self):
        self._name = get_node_handler().name
        while get_node_handler().uri is None:
            time.sleep(0.01) 
        self._uri = get_node_handler().uri

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

    """
    Publisher functions
    Subscirber functions
    Service functions
    """
    def registerPublisher(self, name, topic, datatype, uri):
        """
        registerPublisher
        """
        data = self._generate_message("registerPublisher")
        data[TOPIC_NAME] = topic
        data[TOPIC_TYPE] = datatype
        self._send(self._encode(data))

        if topic not in self._pubs:
            self._pubs[topic] = set()
        self._pubs[topic].add(uri)
        return 1, "Registered [%s] as publisher of [%s]" % (name, topic), []

    def unregisterPublisher(self, name, topic, uri):
        """
        unregisterPublisher
        """
        data = self._generate_message("unregisterPublisher")
        data[NODE_NAME] = name
        data[TOPIC_NAME] = topic
        data[TOPIC_URI] = uri
        self._send(self._encode(data))
        try:
            self._pubs[topic].remove(uri)
        except KeyError:
            pass
        return 1, "unregisterPublisher" ,0

    def registerSubscriber(self, name, topic, datatype, uri):
        """
        registerSubscriber
        """
        data = self._generate_message("registerSubscriber")
        data[TOPIC_NAME] = topic
        data[TOPIC_TYPE] = datatype
        self._send(self._encode(data))
        if topic not in self._subs:
            self._subs[topic] = set()
        self._subs[topic].add(uri)

        if topic in self._pub_history:
            for data in self._pub_history[topic]:
                self._registerPublisherCallback(self._unpack_msg(data))
        return 1, "Subscribed to [%s]" % topic, []

    def unregisterSubscriber(self, name, topic, uri):
        """
        unregisterSubscriber
        """
        data = self._generate_message("unregisterSubscriber")
        data[NODE_NAME] = name
        data[TOPIC_NAME] = topic
        data[TOPIC_URI] = uri
        self._send(self._encode(data))
        try:
            self._subs[topic].remove(data[XMLRPC_URI])
        except KeyError:
            pass
        return 1, "unregisterSubscriber" ,0

    def registerService(self, name, service_name, service_uri, uri):
        """
        registerService
        """
        self._service_cache[service_name] = service_uri
        data = self._generate_message("registerService")
        data[SERVICE_NAME] = service_name
        data[SERVICE_URI] = service_uri
        self._send(self._encode(data))
        return 1, "Registered [%s] as provider of [%s] with service_uri[%s]" % \
            (name, service_name, service_uri), 1

    def unregisterService(self, name, service_name, service_uri):
        """
        unregisterService
        """
        try:
            del self._service_cache[service_name]
        except KeyError:
            pass
        data = self._generate_message("unregisterService")
        data[NODE_NAME] = name
        data[SERVICE_NAME] = service_name
        data[SERVICE_URI] = service_uri
        self._send(self._encode(data))
        return 1, "unregisterService" ,0

    def lookupService(self, caller_id, service_name):
        """
        lookupService
        """
        retry = 5
        while(retry > 0):
            try:
                return 1, "rosrpc URI: [%s]" % self._service_cache[service_name], self._service_cache[service_name]
            except KeyError:
                time.sleep(0.5)
                retry = retry - 1
        return -1, "no provider", ""

    def lookupNode(self, caller_id, node_name):
        """
        lookupNode
        """
        node_info = [data for data in  self._node_info if data[0] == node_name]
        if node_info:
            return 1, "node api", node_info[0][1]
        else:
            return -1, "unknown node" ,None

    def get_pubs(self):
        return self._pubs

    def get_subs(self):
        return self._subs

    """
    Callbacks
    """

    def _registerPublisherCallback(self, data):
        name = data[NODE_NAME]
        topic = data[TOPIC_NAME]
        topic_type = data[TOPIC_TYPE]
        topic_uri = data[XMLRPC_URI]
        uri_list = [topic_uri]

        topic_info = [x for x in self._topic_info if x[0] == topic]
        if topic_info:
            topic_info[0][1] = topic_type
        else:
            self._topic_info.append([topic, topic_type])

        tm = get_topic_manager()
        try:
            tm.lock.acquire()
            if tm.has_subscription(topic):
                self._logger.debug("I has sub topic : %s" % topic)
                get_node_handler().publisherUpdate(name, topic, uri_list)
        except Exception as e:
            self._logger.error(
                    "registerPublisherCallback error is  %s" % e)
            self._logger.error(traceback.format_exc())
        finally:
            tm.lock.release()

        if topic not in self._pubs:
            self._pubs[topic] = set()
        self._pubs[topic].add(topic_uri)

        if topic not in self._pub_history:
            self._pub_history[topic] = set()
        self._pub_history[topic].add(self._encode(data))

    def _unregisterPublisherCallback(self, data):
        name = data[NODE_NAME]
        topic = data[TOPIC_NAME]
        uri = data[XMLRPC_URI]
        uri_list = [uri]

        tm = get_topic_manager()
        try:
            tm.lock.acquire()
            if tm.has_subscription(topic):
                self._logger.debug("I has sub topic, recv unregisSub: %s" % topic)
                get_node_handler().reg_man.reg_removed(topic, uri_list, Registration.SUB)
        except Exception as e:
            self._logger.error(
                "unregisterPublisherCallback error is  %s" % e)
            self._logger.error(traceback.format_exc())
        finally:
            tm.lock.release()

        try:
            self._pubs[topic].remove(uri)
        except KeyError:
            pass

        try:
            self._pub_history[topic].remove(self._encode(data))
        except KeyError:
            pass

    def _registerSubscriberCallback(self, data):
        topic = data[TOPIC_NAME]
        topic_uri = data[XMLRPC_URI]

        if topic not in self._subs:
            self._subs[topic] = set()
        self._subs[topic].add(topic_uri)

    def _unregisterSubscriberCallback(self, data):
        uri = data[XMLRPC_URI]
        topic = data[TOPIC_NAME]
        try:
            self._subs[topic].remove(uri)
        except KeyError:
            pass

    def _registerServiceCallback(self, data):
        service_name = data[SERVICE_NAME]
        service_uri = data[SERVICE_URI]
        self._service_cache[service_name] = service_uri

    def _unregisterServiceCallback(self, data):
        service_name = data[SERVICE_NAME]
        service_uri = data[SERVICE_URI]
        try:
            del self._service_cache[service_name]
        except KeyError:
            pass
 
    def _registerNodeCallback(self, data):
        name = data[NODE_NAME]
        uri = data[XMLRPC_URI]
        node_info = [x for x in self._node_info if x[0] == name]
        if node_info:
            node_info[0][1] = uri
        else:
            self._node_info.append([name, uri])

    def _register_node(self):
        """
        _register_node
        """
        data = self._generate_message("registerNode")
        data[NODE_TIME] = self._node_time
        self._send(self._encode(data))

    """
    Basic functions
    """
    def _generate_message(self, request, timestamp=None):
        if timestamp is None:
            nsec_time = str(int(round(time.time()*1000)))
        else:
            nsec_time = timestamp    
        header = {}
        header[REQUEST_TYPE] = request
        header[NODE_NAME] = self._name
        header[XMLRPC_URI] = self._uri
        header[TIMESTAMP] = nsec_time
        return header

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

    def _encode(self, data):
        return json.dumps(data)

