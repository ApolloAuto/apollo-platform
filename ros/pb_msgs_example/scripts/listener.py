#!/usr/bin/env python

import rospy
import pb_msgs.msg

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.content)

def listener():
    rospy.init_node("listener", anonymous = True)
    
    rospy.Subscriber("pb_chatter", pb_msgs.msg.ShortMessage, callback)

    rospy.spin()

if __name__ == "__main__":
    listener()