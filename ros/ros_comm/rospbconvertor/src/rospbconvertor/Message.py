#!/usr/bin/env python
######################################################
#
#   Message Handle
#
######################################################
"""
Message Handle
"""
import curses
import rospy

import batch_include

class Message(object):
    """
    Message Class
    """

    def __init__(self, name, proto_name, topic, topic_out):
        self.name = name
        self.topic = topic
        self.proto_name = 'batch_include.' + proto_name[:-2]
        self.proto = eval('batch_include.' + proto_name)
        self.topic_out = topic_out

        self.publisher = rospy.Publisher(topic_out, eval(self.proto_name), queue_size=16)

    def callback(self, data):
        """
        callback function
        """
        self.proto.ParseFromString(data.data)
        self.publisher.publish(self.proto)
        #print("recv %s, pub %s " %(self.topic, self.topic_out))
