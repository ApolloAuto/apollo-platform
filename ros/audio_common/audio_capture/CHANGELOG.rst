^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package audio_capture
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.12 (2016-02-29)
-------------------

0.2.11 (2016-02-16)
-------------------

0.2.10 (2016-01-21)
-------------------

0.2.9 (2015-12-02)
------------------
* [audio_capture] add error handler
* [audio_capture] add option to publish captured audio data as wav format
* Fixed memory leak (see `#18 <https://github.com/ros-drivers/audio_common/issues/18>`_).
* Removed trailing whitespace.
* Contributors: Felix Duvallet, Furushchev

0.2.8 (2015-10-02)
------------------
* Update maintainer email
* Contributors: trainman419

0.2.7 (2014-07-25)
------------------
* audio_capture.cpp has to wait for generated AudioData headers
* Contributors: v4hn

0.2.6 (2014-02-26)
------------------
* audio_capture and play _require\_ gstreamer, it's not optional
* Contributors: v4hn

0.2.5 (2014-01-23)
------------------
* "0.2.5"
* Contributors: trainman419

0.2.4 (2013-09-10)
------------------
* Update CMakeLists.txt
* audio_capture: install launchfiles
* Contributors: David Gossow

0.2.3 (2013-07-15)
------------------
* Fix install rule for audio_capture.
* Contributors: Austin Hendrix

0.2.2 (2013-04-10)
------------------

0.2.1 (2013-04-08 13:59)
------------------------

0.2.0 (2013-04-08 13:49)
------------------------
* Finish catkinizing audio_common.
* Catkinize audio_play.
* Catkinize audio_capture.
* Fix typo in package.xml
* Versions and more URLs.
* Convert manifests to package.xml
* Convert audio_capture manifest to package.xml
* Ditch old makefiles.
* Updates manifest
* Updated manifests for rodep2
* oneiric build fixes, bump version to 0.1.6
* Removed redundant thread::thread
* Added a rosdep.yaml file
* Fixed to use audio_common_msgs
* Added ability to use different festival voices
* Updated documentation
* Added ability to capture to file
* Fixed ignore files
* Added hgignore files
* Audio_capture and audio_play working
* Making separate audio_capture and audio_play packages
* Moved audio_transport to audio_capture
* Contributors: Austin Hendrix, Brian Gerkey, Nate Koenig, nkoenig
