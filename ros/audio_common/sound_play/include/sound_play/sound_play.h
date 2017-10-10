/*
 ***********************************************************
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
 ***********************************************************
 */

#ifndef __SOUND_PLAY__SOUND_PLAY__H__
#define __SOUND_PLAY__SOUND_PLAY__H__

#include <string>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sound_play/SoundRequest.h>
#include <boost/thread.hpp>

namespace sound_play
{

/**
 * \brief Class that publishes messages to the sound_play node.
 *
 * This class is a helper class for communicating with the sound_play node
 * via the \ref sound_play::SoundRequest message. It has two ways of being used:
 *
 * - It can create Sound classes that represent a particular sound which
 *   can be played, repeated or stopped.
 *
 * - It provides methods for each way in which the sound_play::SoundRequest
 *   message can be invoked.
 */

class SoundClient
{
public:
	class Sound
	{
		friend class SoundClient;
	private:
		int snd_;
		std::string arg_;
		std::string arg2_;
		SoundClient *client_;

		Sound(SoundClient *sc, int snd, const std::string &arg, const std::string arg2 = std::string())
		{
			client_ = sc;
			snd_ = snd;
			arg_ = arg;
			arg2_ = arg2;
		}

	public:
		/**
		 * \brief Play the Sound.
		 *
		 * This method causes the Sound to be played once.
		 */

		void play()
		{
			client_->sendMsg(snd_, SoundRequest::PLAY_ONCE, arg_, arg2_);
		}

		/**
		 * \brief Play the Sound repeatedly.
		 *
		 * This method causes the Sound to be played repeatedly until stop() is
		 * called.
		 */

		void repeat()
		{
			client_->sendMsg(snd_, SoundRequest::PLAY_START, arg_, arg2_);
		}

		/**
		 * \brief Stop Sound playback.
		 *
		 * This method causes the Sound to stop playing.
		 */

		void stop()
		{
			client_->sendMsg(snd_, SoundRequest::PLAY_STOP, arg_, arg2_);
		}
	};
	
/** \brief Create a SoundClient that publishes on the given topic
 *
 * Creates a SoundClient that publishes to the given topic relative to the
 * given NodeHandle.
 *
 * \param nh Node handle to use when creating the topic.
 *
 * \param topic Topic to publish to.
 */

	SoundClient(ros::NodeHandle &nh, const std::string &topic)
    {
		init(nh, topic);
	}

/** \brief Create a SoundClient with the default topic
 *
 * Creates a SoundClient that publishes to "robotsound".
 */

	SoundClient()
  {
		init(ros::NodeHandle(), "robotsound");
  }

/**
 * \brief Create a voice Sound.
 *
 * Creates a Sound corresponding to saying the indicated text.
 *
 * \param s Text to say
 */

  Sound voiceSound(const std::string &s)
	{
		return Sound(this, SoundRequest::SAY, s);
	}

/**
 * \brief Create a wave Sound.
 *
 * Creates a Sound corresponding to indicated file.
 *
 * \param s File to play. Should be an absolute path that exists on the
 * machine running the sound_play node.
 */

  Sound waveSound(const std::string &s)
	{
		return Sound(this, SoundRequest::PLAY_FILE, s);
	}

/**
 * \brief Create a wave Sound from a package.
 *
 * Creates a Sound corresponding to indicated file.
 *
 * \param p Package containing the sound file.
 * \param s Filename of the WAV or OGG file. Must be an path relative to the package valid
 * on the computer on which the sound_play node is running
 */

  Sound waveSoundFromPkg(const std::string &p, const std::string &s)
	{
		return Sound(this, SoundRequest::PLAY_FILE, s, p);
	}

/**
 * \brief Create a builtin Sound.
 *
 * Creates a Sound corresponding to indicated builtin wave.
 *
 * \param id Identifier of the sound to play.
 */

  Sound builtinSound(int id)
	{
		return Sound(this, id, "");
	}

/** \brief Say a string
 *
 * Send a string to be said by the sound_node. The vocalization can be
 * stopped using stopSaying or stopAll.
 *
 * \param s String to say
 */
	
	void say(const std::string &s, const std::string &voice="voice_kal_diphone")
  {
    sendMsg(SoundRequest::SAY, SoundRequest::PLAY_ONCE, s, voice);
  }

/** \brief Say a string repeatedly
 *
 * The string is said repeatedly until stopSaying or stopAll is used.
 *
 * \param s String to say repeatedly
 */
	
  void repeat(const std::string &s)
  {
    sendMsg(SoundRequest::SAY, SoundRequest::PLAY_START, s);
  }

/** \brief Stop saying a string
 *
 * Stops saying a string that was previously started by say or repeat. The
 * argument indicates which string to stop saying.
 *
 * \param s Same string as in the say or repeat command
 */

  void stopSaying(const std::string &s)
  {
    sendMsg(SoundRequest::SAY, SoundRequest::PLAY_STOP, s);
  }

/** \brief Plays a WAV or OGG file
 *
 * Plays a WAV or OGG file once. The playback can be stopped by stopWave or
 * stopAll.
 *
 * \param s Filename of the WAV or OGG file. Must be an absolute path valid
 * on the computer on which the sound_play node is running
 */

  void playWave(const std::string &s)
  {
    sendMsg(SoundRequest::PLAY_FILE, SoundRequest::PLAY_ONCE, s);
  }

/** \brief Plays a WAV or OGG file repeatedly
 *
 * Plays a WAV or OGG file repeatedly until stopWave or stopAll is used.
 *
 * \param s Filename of the WAV or OGG file. Must be an absolute path valid
 * on the computer on which the sound_play node is running.
 */

  void startWave(const std::string &s)
  {
    sendMsg(SoundRequest::PLAY_FILE, SoundRequest::PLAY_START, s);
  }

/** \brief Stop playing a WAV or OGG file
 *
 * Stops playing a file that was previously started by playWave or
 * startWave.
 *
 * \param s Same string as in the playWave or startWave command
 */

  void stopWave(const std::string &s)
  {
    sendMsg(SoundRequest::PLAY_FILE, SoundRequest::PLAY_STOP, s);
  }

/** \brief Plays a WAV or OGG file from a package
 *
 * Plays a WAV or OGG file once. The playback can be stopped by stopWaveFromPkg or
 * stopAll.
 *
 * \param p Package name containing the sound file.
 * \param s Filename of the WAV or OGG file. Must be an path relative to the package valid
 * on the computer on which the sound_play node is running
 */

  void playWaveFromPkg(const std::string &p, const std::string &s)
  {
    sendMsg(SoundRequest::PLAY_FILE, SoundRequest::PLAY_ONCE, s, p);
  }

/** \brief Plays a WAV or OGG file repeatedly
 *
 * Plays a WAV or OGG file repeatedly until stopWaveFromPkg or stopAll is used.
 *
 * \param p Package name containing the sound file.
 * \param s Filename of the WAV or OGG file. Must be an path relative to the package valid
 * on the computer on which the sound_play node is running
 */

  void startWaveFromPkg(const std::string &p, const std::string &s)
  {
    sendMsg(SoundRequest::PLAY_FILE, SoundRequest::PLAY_START, s, p);
  }

/** \brief Stop playing a WAV or OGG file
 *
 * Stops playing a file that was previously started by playWaveFromPkg or
 * startWaveFromPkg.
 *
 * \param p Package name containing the sound file.
 * \param s Filename of the WAV or OGG file. Must be an path relative to the package valid
 * on the computer on which the sound_play node is running
 */

  void stopWaveFromPkg(const std::string &p, const std::string &s)
  {
    sendMsg(SoundRequest::PLAY_FILE, SoundRequest::PLAY_STOP, s, p);
  }

/** \brief Play a buildin sound
 *
 * Starts playing one of the built-in sounds. built-ing sounds are documented
 * in \ref SoundRequest.msg. Playback can be stopped by stopAll.
 *
 * \param sound Identifier of the sound to play.
 */
	
	void play(int sound)
  {
    sendMsg(sound, SoundRequest::PLAY_ONCE);
  }

/** \brief Play a buildin sound repeatedly
 *
 * Starts playing one of the built-in sounds repeatedly until stop or stopAll 
 * is used. Built-in sounds are documented in \ref SoundRequest.msg.
 *
 * \param sound Identifier of the sound to play.
 */
	
	void start(int sound) 
	{ 
		sendMsg(sound, SoundRequest::PLAY_START); 
	}

/** \brief Stop playing a built-in sound
 *
 * Stops playing a built-in sound started with play or start.
 *
 * \param sound Same sound that was used to start playback.
 */ 

  void stop(int sound)
  {
    sendMsg(sound, SoundRequest::PLAY_STOP);
  }

/** \brief Stop all currently playing sounds
 *
 * This method stops all speech, wave file, and built-in sound playback.
 */

  void stopAll()
	{
		stop(SoundRequest::ALL);
	}
  
  /** \brief Turns warning messages on or off.
	 *  
	 * If a message is sent when no node is subscribed to the topic, a
	 * warning message is printed. This method can be used to enable or
	 * disable warnings.
	 *
	 * \param state True to turn off messages, false to turn them on.
	 */

  void setQuiet(bool state)
	{
		quiet_ = state;
	}

private:
	void init(ros::NodeHandle nh, const std::string &topic)
	{
        nh_ = nh;
		pub_ = nh.advertise<sound_play::SoundRequest>(topic, 5);
		quiet_ = false;
    }

	void sendMsg(int snd, int cmd, const std::string &s = "", const std::string &arg2 = "")
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (!nh_.ok())
      return;

    SoundRequest msg;
    msg.sound = snd;
    msg.command = cmd;
    msg.arg = s;
    msg.arg2 = arg2;
    pub_.publish(msg);

    if (pub_.getNumSubscribers() == 0 && !quiet_)
      ROS_WARN("Sound command issued, but no node is subscribed to the topic. Perhaps you forgot to run soundplay_node.py");
  }

    bool quiet_;
	ros::NodeHandle nh_;
    ros::Publisher pub_;
	boost::mutex mutex_;
};

typedef SoundClient::Sound Sound;

};

#endif
