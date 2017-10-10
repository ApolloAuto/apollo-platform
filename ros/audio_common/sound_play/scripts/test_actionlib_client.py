#! /usr/bin/env python

import roslib; roslib.load_manifest('sound_play')
import rospy
import actionlib
from sound_play.msg import SoundRequest, SoundRequestAction, SoundRequestGoal

import os

def sound_play_client():
    client = actionlib.SimpleActionClient('sound_play', SoundRequestAction)

    client.wait_for_server()

    print "Need Unplugging"
    goal = SoundRequestGoal()
    goal.sound_request.sound = SoundRequest.NEEDS_UNPLUGGING
    goal.sound_request.command = SoundRequest.PLAY_ONCE

    client.send_goal(goal)
    client.wait_for_result()
    print client.get_result()
    print "End Need Unplugging"
    print

    print "Need Plugging"
    goal = SoundRequestGoal()
    goal.sound_request.sound = SoundRequest.NEEDS_PLUGGING
    goal.sound_request.command = SoundRequest.PLAY_ONCE
    client.send_goal(goal)
    client.wait_for_result()
    print client.get_result()
    print "End Need Plugging"
    print

    print "Say"
    goal = SoundRequestGoal()
    goal.sound_request.sound = SoundRequest.SAY
    goal.sound_request.command = SoundRequest.PLAY_ONCE
    goal.sound_request.arg = "Testing the actionlib interface A P I"
    client.send_goal(goal)
    client.wait_for_result()
    print client.get_result()
    print "End Say"
    print

    print "Wav"
    goal = SoundRequestGoal()
    goal.sound_request.sound = SoundRequest.PLAY_FILE
    goal.sound_request.command = SoundRequest.PLAY_ONCE
    goal.sound_request.arg = os.path.join(roslib.packages.get_pkg_dir('sound_play'),'sounds') + "/say-beep.wav"
    client.send_goal(goal)
    client.wait_for_result()
    print client.get_result()
    print "End wav"
    print

if __name__ == '__main__':
    rospy.init_node('soundplay_client_test')
    sound_play_client()
