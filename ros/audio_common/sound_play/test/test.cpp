/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <sound_play/sound_play.h>
#include <unistd.h>

void sleepok(int t, ros::NodeHandle &nh)
{
  if (nh.ok())
		sleep(t);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sound_play_test");

  ros::NodeHandle nh;
  sound_play::SoundClient sc;

  sleepok(1, nh);
  
  while(nh.ok())
  {
    sc.say("Hello world!");
    sleepok(2, nh);

    const char *str1 = "I am annoying.";
    sc.repeat(str1);
    sleepok(4, nh);
    sc.stopSaying(str1);

    sc.playWave("/usr/share/xemacs21/xemacs-packages/etc/sounds/boing.wav");
    sleepok(2, nh);

    const char *str2 = "/usr/share/xemacs21/xemacs-packages/etc/sounds/piano-beep.wav";
    sc.startWave(str2);
    sleepok(4, nh);
    sc.stopWave(str2);

    sc.play(sound_play::SoundRequest::NEEDS_UNPLUGGING);
    sleepok(2, nh);

    sc.start(sound_play::SoundRequest::BACKINGUP);
    sleepok(4, nh);
    sc.stop(sound_play::SoundRequest::BACKINGUP);

		sleepok(2, nh);
		sound_play::Sound s1 = sc.waveSound("/usr/share/xemacs21/xemacs-packages/etc/sounds/boing.wav");
		s1.repeat();
		sleepok(1, nh);
		s1.stop();
		
		sleepok(2, nh);
		sound_play::Sound s2 = sc.voiceSound("This is a really long sentence that will get cut off.");
		s2.play();
		sleepok(1, nh);
		s2.stop();

		sleepok(2, nh);
		sound_play::Sound s3 = sc.builtinSound(sound_play::SoundRequest::NEEDS_UNPLUGGING_BADLY);
		s3.play();
		sleepok(1, nh);
		s3.stop();

		sleepok(2, nh);
		sound_play::Sound s4 = sc.waveSoundFromPkg("sound_play", "sounds/BACKINGUP.ogg");
		s4.play();
		sleepok(1, nh);
		s4.stop();
  }
}
