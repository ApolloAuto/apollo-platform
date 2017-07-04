#!/usr/bin/env python

import os
import random
import rospy
import threading
import traceback
from std_msgs.msg import String

from Message import Message

primitive = (int, str, bool, unicode)

class Convertor(object):
    """
    Plotter Class
    """

    def __init__(self):
        # topic
        self.META_DATA_FILE = os.path.join(os.path.dirname(__file__), 'meta.data')
        self.messages = []

        try:
            with open(self.META_DATA_FILE) as f:
                for line in f:
                    module_name, proto_name, topic, topic_out = line.strip().split(' ')
                    if module_name == 'MODULE' or proto_name == '#' or\
                        topic == '#' or topic_out == '#':
                        pass
                    else:
                        cur_message = Message(module_name, proto_name,
                                                 topic, topic_out)
                        self.messages.append(cur_message)
        except Exception as e:
            print '{0} open failed! with error: {1}'.format(self.META_DATA_FILE, e.message)
            exit(1)

def main():
    """
    Main function
    """
    rospy.init_node('pb_convertor', anonymous=True)

    cov = Convertor()
    sublist = []
    for msg in cov.messages:
        sublist.append(rospy.Subscriber(msg.topic, String, msg.callback, queue_size=16))

    rospy.spin()

def rospbconvert_main():
    try:
        main()
    except Exception as e:
        tb = traceback.format_exc()
        print tb

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        tb = traceback.format_exc()
        print tb
