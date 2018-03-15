^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package theora_image_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.5 (2016-10-03)
------------------

1.9.4 (2016-10-02)
------------------
* address gcc6 build error and tune
  With gcc6, compiling fails with `stdlib.h: No such file or directory`,
  as including '-isystem /usr/include' breaks with gcc6, cf.,
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70129.
  This commit addresses this issue for this package in the same way
  it was addressed in various other ROS packages. A list of related
  commits and pull requests is at:
  https://github.com/ros/rosdistro/issues/12783
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@oss.bmw-carit.de>
* Contributors: Lukas Bulwahn

1.9.3 (2016-01-17)
------------------
* remove useless tf dependencies
* Contributors: Vincent Rabaud

1.9.2 (2015-04-25)
------------------
* get code to compile with OpenCV3
* Contributors: Vincent Rabaud

1.9.1 (2014-07-18)
------------------
* Some cleanup in package.xml and CMakeLists.txt
  - builds broke sporadically (I think because of the missing *_gencpp in
  add_dependencies) with missing Packet.h file.
  - I’m no catkin expert, but these changes make catkin_lint happy (no
  more errors at least).
* Contributors: Nikolaus Demmel

1.9.0 (2014-05-16)
------------------
* remove __connection_header for indigo see http://github.com/ros-perception/image_transport_plugins.git
* Contributors: Kei Okada

1.8.21 (2013-06-27)
-------------------

1.8.20 (2013-03-18)
-------------------
* 1.8.19 -> 1.8.20
* fixing missing theoraenc and theoradec library links
* Contributors: Julius Kammerl

1.8.19 (2013-02-24)
-------------------
* 1.8.18 -> 1.8.19
* Contributors: Julius Kammerl

1.8.18 (2013-02-07 17:59)
-------------------------
* 1.8.17 -> 1.8.18
* fixing input format checks (enabling rgba, bgra) + minor fixes
* Contributors: Julius Kammerl

1.8.17 (2013-01-18)
-------------------
* 1.8.16 -> 1.8.17
* fixed color conversion bug in theora_image_transport
* Contributors: Julius Kammerl

1.8.16 (2013-01-17)
-------------------
* 1.8.15 -> 1.8.16
* use the pluginlib script to remove some runtime warnings
* Contributors: Julius Kammerl, Vincent Rabaud

1.8.15 (2012-12-28 20:11)
-------------------------

1.8.14 (2012-12-28 20:02)
-------------------------

1.8.13 (2012-12-28 19:06)
-------------------------
* fix the bad exports
* make sure the plugins are visible by image_transport
* added license headers to various cpp and h files
* Contributors: Aaron Blasdel, Vincent Rabaud

1.8.12 (2012-12-19 19:30)
-------------------------
* fix downstream stuff in cmake
* Contributors: Dirk Thomas

1.8.11 (2012-12-19 17:17)
-------------------------
* fix cmake order
* Contributors: Dirk Thomas

1.8.10 (2012-12-19 17:03)
-------------------------
* fix dyn reconf
* Contributors: Dirk Thomas

1.8.9 (2012-12-19 00:26)
------------------------
* switching to verion 1.8.9
* fixing dynamic_reconfigure related catkin errors
* Contributors: Julius Kammerl

1.8.8 (2012-12-17)
------------------
* more message generation related catkin changes
* adding message_runtime deb to CMakeLists.txt
* adding build_deb on message_generation & mrun_deb on message_runtime
* Updated package.xml for new buildtool_depend tag for catkin requirement
* Contributors: Julius Kammerl, mirzashah

1.8.7 (2012-12-10 15:29)
------------------------
* adding missing tf build dependency
* Contributors: Julius Kammerl

1.8.6 (2012-12-10 15:08)
------------------------
* switching to version 1.8.6
* Contributors: Julius Kammerl

1.8.5 (2012-12-09)
------------------
* adding missing build debs
* added class_loader_hide_library_symbols macros to CMakeList
* switching to 1.8.5
* Contributors: Julius Kammerl

1.8.4 (2012-11-30)
------------------
* switching to version 1.8.4
* catkinizing theora_image_transport
* adding plugin.xml exports for pluginlib
* catkinizing theora_image_transport
* github migration from code.ros.org (r40053)
* theora_image_transport: Restored build of ogg_saver, though it really needs more work to be robust.
* theora_image_transport: Removed debug output.
* theora_image_transport: Renamed compressed_plugins.xml to theora_plugins.xml.
* theora_image_transport: Added migration rule for new Packet message.
* image_transport_plugins: Updated manifests to have better summaries, correct URLs.
* theora: Fixed export flags of libtheora. No longer need hack in theora_image_transport's CMakeLists. Temporarily disabled building ogg_saver.
* theora_image_transport: Copy connection header into the output Image.
* theora_image_transport: Publisher sends new headers if image size changes. Better error handling in publisher. Always turn off latching.
* theora_image_transport: Subscriber ignores delta frames until it gets a keyframe. Gets rid of junk frames at the beginning.
* theora_image_transport: Properly clear everything before receiving new headers, which now works without error on the subscriber side.
* theora_image_transport: Better error handling. Support for receiving new headers in subscriber. Handle duplicate frames correctly. Fixed a couple memory leaks.
* theora_image_transport: Force queue_size to be big enough for the headers on both ends. Got rid of sleeps after publishing header packets. More code cleanup.
* theora_image_transport: Added ROS header to Packet msg, fixing `#3882 <https://github.com/ros-perception/image_transport_plugins/issues/3882>`_. Fixed reception of comment header and now properly detect when all headers received.
* theora_image_transport: Pull out original (non-padded) region in subscriber.
* theora_image_transport: Cleaned up encoding/decoding to make good use of existing OpenCV functions. Partially fixed `#3082 <https://github.com/ros-perception/image_transport_plugins/issues/3082>`_, poor handling of oddly-sized images.
* theora_image_transport: Cleanup of TheoraPublisher.
* Added Ubuntu platform tags to manifest
* Adding ogg_saver node to dump a theora stream to a .ogg file playable in VLC, mplayer, etc
* Fixing bug (typo) where theora_publisher always set target bitrate to 1.  I'm surprised it was working at all.
* Remove use of deprecated rosbuild macros
* Switch to opencv2
* Ooops, segfault
* Hopefully fixed a theora_subscriber bug, Patrick will test.
* theora_image_transport: Override getTransportName().
* Updating theora_image_transport to work with the latest image_transport API
* Removed explicit library prefix and suffix
* image_transport_plugins: Initial stack check-in. Includes theora_image_transport, compressed_image_transport and libtheora. Currently depends on opencv, but may excise this in the future.
* Contributors: Julius Kammerl, ethan, gerkey, jamesb, mihelich, pmihelich, wheeler
