#!/usr/bin/env python

"""
Simple example showing how to use the SoundClient provided by libsoundplay,
in blocking, non-blocking, and explicit usage.
"""

import rospy
from sound_play.libsoundplay import SoundClient
from sound_play.msg import SoundRequest


def play_explicit():
    rospy.loginfo('Example: SoundClient play methods can take in an explicit'
                  ' blocking parameter')
    soundhandle = SoundClient()  # blocking = False by default
    rospy.sleep(0.5)  # Ensure publisher connection is successful.

    sound_beep = soundhandle.waveSound("say-beep.wav", volume=0.5)
    # Play the same sound twice, once blocking and once not. The first call is
    # blocking (explicitly specified).
    sound_beep.play(blocking=True)
    # This call is not blocking (uses the SoundClient's setting).
    sound_beep.play()
    rospy.sleep(0.5)  # Let sound complete.

    # Play a blocking sound.
    soundhandle.play(SoundRequest.NEEDS_UNPLUGGING, blocking=True)

    # Create a new SoundClient where the default behavior *is* to block.
    soundhandle = SoundClient(blocking=True)
    soundhandle.say('Say-ing stuff while block-ing')
    soundhandle.say('Say-ing stuff without block-ing', blocking=False)
    rospy.sleep(1)


def play_blocking():
    """
    Play various sounds, blocking until each is completed before going to the
    next.
    """
    rospy.loginfo('Example: Playing sounds in *blocking* mode.')
    soundhandle = SoundClient(blocking=True)

    rospy.loginfo('Playing say-beep at full volume.')
    soundhandle.playWave('say-beep.wav')

    rospy.loginfo('Playing say-beep at volume 0.3.')
    soundhandle.playWave('say-beep.wav', volume=0.3)

    rospy.loginfo('Playing sound for NEEDS_PLUGGING.')
    soundhandle.play(SoundRequest.NEEDS_PLUGGING)

    rospy.loginfo('Speaking some long string.')
    soundhandle.say('It was the best of times, it was the worst of times.')


def play_nonblocking():
    """
    Play the same sounds with manual pauses between them.
    """
    rospy.loginfo('Example: Playing sounds in *non-blocking* mode.')
    # NOTE: you must sleep at the beginning to let the SoundClient publisher
    # establish a connection to the soundplay_node.
    soundhandle = SoundClient(blocking=False)
    rospy.sleep(1)

    # In the non-blocking version you need to sleep between calls.
    rospy.loginfo('Playing say-beep at full volume.')
    soundhandle.playWave('say-beep.wav')
    rospy.sleep(1)

    rospy.loginfo('Playing say-beep at volume 0.3.')
    soundhandle.playWave('say-beep.wav', volume=0.3)
    rospy.sleep(1)

    rospy.loginfo('Playing sound for NEEDS_PLUGGING.')
    soundhandle.play(SoundRequest.NEEDS_PLUGGING)
    rospy.sleep(1)

    rospy.loginfo('Speaking some long string.')
    soundhandle.say('It was the best of times, it was the worst of times.')
    # Note we will return before the string has finished playing.


if __name__ == '__main__':
    rospy.init_node('soundclient_example', anonymous=False)
    play_explicit()
    play_blocking()
    play_nonblocking()
    rospy.loginfo('Finished')
