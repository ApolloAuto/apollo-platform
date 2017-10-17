#!/usr/bin/env python

import unittest

import rospy
import rostest
from sound_play.libsoundplay import SoundClient

class TestCase(unittest.TestCase):
    def test_soundclient_constructor(self):
        s = SoundClient()
        self.assertIsNotNone(s)

if __name__ == '__main__':
    rostest.rosrun('sound_play', 'test_sound_client', TestCase)

__author__ = 'Felix Duvallet'
