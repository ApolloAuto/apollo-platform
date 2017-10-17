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


import sys

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--help':
        print 'Usage: %s \'String to say.\''%sys.argv[0]
        print '       %s < file_to_say.txt'%sys.argv[0]
        print
        print 'Says a string. For a string on the command line, you must use quotes as'
        print 'appropriate. For a string on standard input, the command will wait for'
        print 'EOF before saying anything.'
        exit(-1)

    # Import after printing usage for speed.
    import rospy
    from sound_play.msg import SoundRequest
    from sound_play.libsoundplay import SoundClient

    if len(sys.argv) == 1:
        print 'Awaiting something to say on standard input.'

    # Ordered this way to minimize wait time.
    rospy.init_node('say', anonymous = True)
    soundhandle = SoundClient()
    rospy.sleep(1)

    voice = 'voice_kal_diphone'

    if len(sys.argv) == 1:
        s = sys.stdin.read()
    else:
        s = sys.argv[1]

        if len(sys.argv) > 2:
            voice = sys.argv[2]

    print 'Saying: %s' % s
    print 'Voice: %s' % voice

    soundhandle.say(s, voice)
    rospy.sleep(1)
