^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package audio_play
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.12 (2016-02-29)
-------------------

0.2.11 (2016-02-16)
-------------------

0.2.10 (2016-01-21)
-------------------

0.2.9 (2015-12-02)
------------------

0.2.8 (2015-10-02)
------------------
* Changed message level to warning
* Fixed underflow.
  Before the sink buffer underflows the pipeline is paused. When data is received again the pipeline is set to playing again.
* Change audio sink to autoaudiosink
* Update maintainer email
* Contributors: Benny, Hans Gaiser, trainman419

0.2.7 (2014-07-25)
------------------

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

0.2.3 (2013-07-15)
------------------
* Fix dependencies and install rules.
* Contributors: Austin Hendrix

0.2.2 (2013-04-10)
------------------

0.2.1 (2013-04-08 13:59)
------------------------

0.2.0 (2013-04-08 13:49)
------------------------
* Finish catkinizing audio_common.
* Catkinize audio_play.
* Fix typo in package.xml
* Versions and more URLs.
* Convert manifests to package.xml
* Ditch old makefiles.
* Updates manifest
* Updated manifests for rodep2
* oneiric build fixes, bump version to 0.1.6
* Removed another duplicate thread::thread
* Added a rosdep.yaml file
* Fixed to use audio_common_msgs
* Added ability to use different festival voices
* Updated documentation
* Update to audio_play
* Fixed ignore files
* Added hgignore files
* Audio_capture and audio_play working
* Making separate audio_capture and audio_play packages
* Contributors: Austin Hendrix, Brian Gerkey, Nate Koenig, nkoenig
