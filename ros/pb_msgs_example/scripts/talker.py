#!/usr/bin/env python

import rospy
import pb_msgs.msg

def talker():
    pub = rospy.Publisher('pb_chatter', pb_msgs.msg.ShortMessage, queue_size = 10)
    rospy.init_node('talker', anonymous = True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = pb_msgs.msg.ShortMessage()
        now = rospy.get_rostime()
        msg.stamp.sec = now.secs
        msg.stamp.nsec = now.nsecs
        msg.content = "Hello world!"
        pub.publish(msg)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass