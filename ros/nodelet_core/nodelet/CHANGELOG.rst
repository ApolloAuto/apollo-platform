^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package nodelet
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.9.10 (2017-03-27)
-------------------
* installs the list_nodelets script (`#58 <https://github.com/ros/nodelet_core/issues/58>`_)
  * python3 compatibility
  * pep8
  * install list_nodelets
  * print message with service name
* return outside of try catch
* fix unused var warning
* give node a name, empty node names not supported since https://github.com/ros/ros_comm/commit/bd3af70520648783da8aa4d3610f234a4d2bd41f
* remove tabs
* fix help message
* Contributors: Mikael Arguedas

1.9.9 (2017-02-17)
------------------
* drop unused dependency on tinyxml (`#54 <https://github.com/ros/nodelet_core/pull/54>`_)
* Install the declared_nodelets script (`#53 <https://github.com/ros/nodelet_core/pull/53>`_)
* Contributors: Dmitry Rozhkov, Kentaro Wada

1.9.8 (2016-11-15)
------------------
* Fix bond handling during nodelet unloading (`#51 <https://github.com/ros/nodelet_core/issues/51>`_)
  * add test whether bond breaking on unload works (tests `#50 <https://github.com/ros/nodelet_core/issues/50>`_)
  * disable callback for broken bond when we are breaking it
  This avoids the nodelet::LoaderROS::unload() method to be called
  twice for the same nodelet, causing an error output.
  * use AsyncSpinner for nodelet load in order for the shutdown procedure to work
  During shutdown, the bonds still need to communicate their status in order
  for the nodelet to properly/cleanly/quickly unload. This requires the node
  to spin.
  * add test whether LoaderROS::unload() is called twice (tests `#50 <https://github.com/ros/nodelet_core/issues/50>`_)
* Contributors: Daniel Seifert

1.9.7 (2016-10-24)
------------------
* Use rospkg instead of roslib in declared_nodelets
  Close https://github.com/ros/nodelet_core/issues/48
* Contributors: Kentaro Wada

1.9.6 (2016-09-20)
------------------

1.9.5 (2016-06-22)
------------------
* more sane debugging messages
* Contributors: Daniel Stonier

1.9.4 (2016-03-15)
------------------
* update maintainer
* Contributors: Mikael Arguedas

1.9.3 (2015-08-05)
------------------
* adding support for named nodelet loggers
* nodelet loader: display error messages from both load attempts on failure
* Contributors: Max Schwarz, Tully Foote

1.9.2 (2014-10-30)
------------------
* Fix dependency version
* Contributors: Esteve Fernandez

1.9.1 (2014-10-29)
------------------
* Use FindUUID.cmake from cmake-modules to find the UUID libraries
* nodelet: Loader: do not call impl->refresh_classes_ if not available
* Contributors: Esteve Fernandez, Max Schwarz

1.9.0 (2014-06-16)
------------------
* Fix initialization error handling (`#13 <https://github.com/ros/nodelet_core/issues/13>`_)
* Contributors: Esteve Fernandez

1.8.3 (2014-05-08)
------------------
* Add version to pluginlib dependency
* nodelet: avoid breaking bond when unloading unknown nodelet
* nodelet: refresh list of available classes if class is not found
* Fixed missing header
* Correctly check that there are enough arguments when nodelet is launched with the unload command
* Exit if Loader::load returns failure in "standalone" mode instead of continuing to run
* Contributors: Dirk Thomas, Esteve Fernandez, Forrest Voight, Gary Servin, Marcus Liebhardt, Mitchell Wills

* fix missing header (`#14 <https://github.com/ros/nodelet_core/issues/14>`_)
* fix check that there are enough arguments when nodelet is launched with the unload command (`#12 <https://github.com/ros/nodelet_core/issues/12>`_)
* exit if Loader::load returns failure in "standalone" mode instead of continuing to run (`#11 <https://github.com/ros/nodelet_core/issues/11>`_)

1.8.2 (2014-01-07)
------------------
* fix erasing bond when it breaks (`#8 <https://github.com/ros/nodelet_core/issues/8>`_)

1.8.0 (2013-07-11)
------------------
* add missing archive/library/runtime destination for library
* Export pluginlib as a transitive dependency
  Also remove some old Apple specific rules which
  are no longer required.
* use EXPORTED_TARGETS variable instead of explicit target names
* update email in package.xml

1.7.15 (2013-03-12)
-------------------

1.7.14 (2013-01-13)
-------------------
* add missing link library uuid (fix `#4 <https://github.com/ros/nodelet_core/issues/4>`_)

1.7.13 (2012-12-27)
-------------------
* move nodelet_topic_tools to separate package, fix unit tests

1.7.12 (2012-12-19 01:34)
-------------------------

1.7.11 (2012-12-19 00:58)
-------------------------

1.7.10 (2012-12-14)
-------------------
* add missing dep to catkin

1.7.9 (2012-12-13)
------------------
* add missing downstream depend
* switched from langs to message_* packages

1.7.8 (2012-12-06)
------------------
* updated catkin_package(DEPENDS)

1.7.7 (2012-11-01)
------------------

1.7.6 (2012-10-30)
------------------
* fix catkin function order
* clean up package.xml files

1.7.5 (2012-10-23)
------------------

1.7.4 (2012-10-08)
------------------

1.7.3 (2012-10-04)
------------------

1.7.2 (2012-10-03)
------------------

1.7.1 (2012-10-02)
------------------
* adding nodelet_core metapackage and reving to 1.7.1

1.7.0 (2012-10-01)
------------------
* fix dependencies
* make it compile locally
* first pass at catkinizing the stack
* updated to latest pluginlib
* updated usage of pluginlib according to updated REP 121
* use updated pluginlib to auto-unload libraries when unloading nodelets
* fixed issue `#5144 <https://github.com/ros/nodelet_core/issues/5144>`_ on OS X lion
* Commentary on who owns what among Loader, Nodelet, CallbackQueue and CallbackQueueManager.
* Moved most of Loader's member variables into an opaque PIMPL struct so we can change things without breaking ABI.
* All bond code moved to LoaderROS. Loader no longer needs to know about bond.
* Removed CallbackQueue::disable(). Loader removes a nodelet's queues from the queue manager when unloading it, which is sufficient to prevent new callbacks for that nodelet getting added.
* Removed some code and comments concerned with callbacks getting called after their nodelet's destruction. This can't actually happen anymore, since callbacks only fire if they can lock a weak_ptr to their parent nodelet.
* Refactoring to streamline Nodelet back down to a simple plugin interface. It no longer knows about detail::CallbackQueue[Manager] or Bond; init() simply takes the single- and multi-threaded ros::CallbackQueueInterface* instead (defaulting to NULL). Loader owns the callback queues and bond for each nodelet. This makes it possible to use Nodelet without all the surrounding infrastructure.
* Take Bond pointers as const-ref instead of value in Loader and Nodelet.
* Added Loader constructor taking a boost::function object used as a factory for
  nodelet instances, replacing the default use of a pluginlib class loader. This
  is to support ROSGUI, which defines its specialized plugin interface as a
  subclass of Nodelet, and thus needs a different class loader.
* Removed some debug code in Loader constructor.
* 'nodelet load' more reliably unloads the nodelet on exiting. In particular it intercepts XML-RPC shutdown command, used for example by 'rosnode kill'.
* Enabled error output when service calls fail abnormally.
* Rewrote tracked_object logic to be clearer.
* Have detail::CallbackQueue use a ros::VoidConstWPtr as the tracked object, which is now optional. More generic, and fixes test_nodelet which was broken by the last commit.
* Fixed race conditions/deadlocks when unloading a nodelet. Now disable the nodelet's callback queues before deleting it. The queues have a WPtr to the nodelet, so any outstanding callbacks will get discarded.
* ~Loader now stops callback manager threads before destroying the nodelets. Otherwise the worker threads could operate on nodelet data as/after it's destroyed.
* Use ros::names::parentNamespace().
* Cleaned scoped_ptr's out of ThreadInfo and updated its padding.
* Made ThreadInfo::calling an atomic_count. This allows the manager thread to pick the queue with least work more accurately, and reduces contention b/c getSmallestQueue no longer needs to lock on ``queue_mutex_``.
* Minor code cleanup and finer locking in managerThread().
* Actually pad ThreadInfo to a multiple of 64 bytes. Previous expression was wrongly wrapped in sizeof().
* Instead of ``thread_info_``.resize(num_threads), push each ThreadInfo on individually. With resize(), all threads ended up sharing the same queue_mutex and queue_cond. Doesn't seem to be much of a performance win though.
* Added test instrumentation to CallbackQueueManager to track size of worker thread queues over time. Must be enabled at compilation time with -DNODELET_QUEUE_DEBUG.
* nodelet patches for osx lion support from wjwwood
* Added --no-bond option to nodelet loading to disable bonds.
* updated platform tags
* don't need to link against tinyxml directly
* link against system tinyxml
* Fix for `#4855 <https://github.com/ros/nodelet_core/issues/4855>`_
  This fix actually makes sense, but that it wasn't caught earlier
  doesn't.  The construction of
  nodelet::Loader n(false)
  was creating the first node handle and letting it go out of scope,
  which was automagically calling ros::shutdown(), which is a dumb thing
  for ros::NodeHandle to do automagically on destruction.
* Each nodelet now places its bonds on a custom callback queue
* a script to list declared nodelets
* real fix for `#4460 <https://github.com/ros/nodelet_core/issues/4460>`_
* patch for `#4460 <https://github.com/ros/nodelet_core/issues/4460>`_
* adding support for once, throttle, and filter features.  With unit tests for all but the filters `#4681 <https://github.com/ros/nodelet_core/issues/4681>`_
* fix for `#4609 <https://github.com/ros/nodelet_core/issues/4609>`_
* MUX simplified by using a 8-connected null filters
  DeMUX has a specialization for message type (uses ros::Subscriber internally by default)
  Added rosdep for nodelet (uuid)
* adding optional namespace aware constructor to nodelet loader. `#4243 <https://github.com/ros/nodelet_core/issues/4243>`_ and fixing vestigial comments referencing Filters `#4221 <https://github.com/ros/nodelet_core/issues/4221>`_
* nodelet uses bond to handle crashes on the manager or the spawner end.  `#4221 <https://github.com/ros/nodelet_core/issues/4221>`_
* locking in all cases
* fix hang on CallbackQueueManager destruction (`#4402 <https://github.com/ros/nodelet_core/issues/4402>`_)
* better check for services
* fix hanging tests and a hang on nodelet CallbackQueueManager destruction (`#4082 <https://github.com/ros/nodelet_core/issues/4082>`_)
* added a boost mutex
* preventing nodelets from busywaiting
* Added optional parameter num_worker_threads to nodelets.
* Added Ubuntu platform tags to manifest
* implemented nodelet unloading on shutdown
* fixed a segfault on destroy
* merging josh's branch from ticket `#3875 <https://github.com/ros/nodelet_core/issues/3875>`_
* adding usage
* fancy new command line parsing for nodelets `#3876 <https://github.com/ros/nodelet_core/issues/3876>`_
* moving topic tools out of nodelet proper, removing rospy and message_filters dependencies from nodelet
* doc updates
* fixed a segfault
* small changes (ptr->boost shared_ptr)
* init guard
* making nodehandles pointers to avoid default constructors
* switching mt_spinner to be a pointer created on init so it's not trying to create a nodehandle at construction
* cleanup
* switching to cpp command based nodelet implementation as per API review
* changes as per API review
* enforcing unique name in manager
* supporting argv passing on server side
* getname return type for API review
* adding MT Nodehandle creation methods and fixing up tutorials
* passing parameters
* we're always going to spin
* added my_args in the service call
* some changes as we discuss them during the API review
* cleaning up private and public api elements
* nodelet_internal_init is now private and a friend of NodeletLoader
* nodelet API changes
* COND rosconsole Nodelet wrappers working
* adding multithreaded callback queue
* removing unnecessary code after refactor
* adding NODELET rosconsole wrappers, note init method is now void args
* moving nodelet package into common trunk so I don't lose it in reorganization
