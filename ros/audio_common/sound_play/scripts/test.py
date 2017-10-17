#!/usr/bin/env python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

# Author: Blaise Gassend

import rospy, os, sys
from sound_play.msg import SoundRequest

from sound_play.libsoundplay import SoundClient

def sleep(t):
    try:
        rospy.sleep(t)
    except:
        pass

if __name__ == '__main__':
    rospy.init_node('soundplay_test', anonymous = True)
    soundhandle = SoundClient()

    rospy.sleep(1)
    
    soundhandle.stopAll()

    print "This script will run continuously until you hit CTRL+C, testing various sound_node sound types."

    print
    #print 'Try to play wave files that do not exist.'
    #soundhandle.playWave('17')
    #soundhandle.playWave('dummy')
        
    #print 'say'
    #soundhandle.say('Hello world!')
    #sleep(3)
    #    
    print 'wave'
    soundhandle.playWave('say-beep.wav')
    sleep(2)
        
    print 'plugging'
    soundhandle.play(SoundRequest.NEEDS_PLUGGING)
    sleep(2)

    print 'unplugging'
    soundhandle.play(SoundRequest.NEEDS_UNPLUGGING)
    sleep(2)

    print 'plugging badly'
    soundhandle.play(SoundRequest.NEEDS_PLUGGING_BADLY)
    sleep(2)

    print 'unplugging badly'
    soundhandle.play(SoundRequest.NEEDS_UNPLUGGING_BADLY)
    sleep(2)

    s1 = soundhandle.builtinSound(SoundRequest.NEEDS_UNPLUGGING_BADLY)
    s2 = soundhandle.waveSound("say-beep.wav")
    s3 = soundhandle.voiceSound("Testing the new A P I")

    print "New API start voice"
    s3.repeat()
    sleep(3)

    print "New API wave"
    s2.play()
    sleep(2)

    print "New API builtin"
    s1.play()
    sleep(2)

    print "New API stop"
    s3.stop()
