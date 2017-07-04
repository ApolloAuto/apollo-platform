tf2
----------------

.. currentmodule:: tf2


.. exception:: TransformException

    base class for tf exceptions.  Because :exc:`tf2.TransformException` is the
    base class for other exceptions, you can catch all tf exceptions
    by writing::

        try:
            # do some tf2 work
        except tf2.TransformException:
            print "some tf2 exception happened"
        

.. exception:: ConnectivityException

   subclass of :exc:`TransformException`.
   Raised when that the fixed_frame tree is not connected between the frames requested.

.. exception:: LookupException

   subclass of :exc:`TransformException`.
   Raised when a tf method has attempted to access a frame, but
   the frame is not in the graph.
   The most common reason for this is that the frame is not
   being published, or a parent frame was not set correctly 
   causing the tree to be broken.  

.. exception:: ExtrapolationException

   subclass of :exc:`TransformException`
   Raised when a tf method would have required extrapolation beyond current limits.


.. exception:: InvalidArgumentException

   subclass of :exc:`TransformException`.
   Raised when the arguments to the method are called improperly formed.  An example of why this might be raised is if an argument is nan. 


BufferCore
-----------

.. class:: tf2.BufferCore(cache_time = rospy.Duration(10))

   :param cache_time: how long the buffer should retain transformation information in the past.

   The BufferCore object is the core of tf2. It maintains a
   time-varying graph of transforms, and permits asynchronous graph
   modification and queries:

   .. doctest::

       >>> import rospy
       >>> import tf2
       >>> import geometry_msgs.msg
       >>> t = tf2.BufferCore(rospy.Duration(10.0))
       >>> t.getFrameStrings()
       []
       >>> m = geometry_msgs.msg.TransformStamped()
       >>> m.header.frame_id = 'THISFRAME'
       >>> m.child_frame_id = 'CHILD'
       >>> m.transform.translation.x = 2.71828183
       >>> m.transform.rotation.w = 1.0
       >>> t.setTransform(m)
       >>> t.getFrameStrings()
       ['/CHILD', '/THISFRAME']
       >>> t.lookupTransform('THISFRAME', 'CHILD', rospy.Time(0))
       ((2.71828183, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
       >>> t.lookupTransform('CHILD', 'THISFRAME', rospy.Time(0))
       ((-2.71828183, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


   The transformer refers to frames using strings, and represents
   transformations using translation (x, y, z) and quaternions (x, y,
   z, w) expressed as Python a :class:`tuple`.
   Transformer also does not mandate any particular
   linear algebra library.
   Transformer does not handle ROS messages directly; the only ROS type it
   uses is `rospy.Time() <http://www.ros.org/doc/api/rospy/html/rospy.rostime-module.html>`_.
   
   To use tf with ROS messages, see :class:`TransformerROS` and :class:`TransformListener`.

    .. method::  allFramesAsYAML() -> string

        Returns a string representing all frames, intended for debugging tools.

    .. method::  allFramesAsString() -> string

        Returns a human-readable string representing all frames

    .. method::  setTransform(transform, authority = "")

        :param transform: transform object, see below
        :param authority: string giving authority for this transformation.

        Adds a new transform to the Transformer graph. transform is an object with the following structure::

            header
                stamp             time stamp, rospy.Time
                frame_id          string, parent frame
            child_frame_id        string, child frame
            transform
                translation
                    x             float
                    y             float
                    z             float
                rotation
                    x             float
                    y             float
                    z             float
                    w             float

        These members exactly match those of a ROS TransformStamped message.

    .. method::  canTransform(target_frame, source_frame, time) -> bool

        :param target_frame: transformation target frame in tf, string
        :param source_frame: transformation source frame in tf, string
        :param time: time of the transformation, use ``rospy.Time()`` to indicate present time.

        Returns True if the Transformer can determine the transform from source_frame to target_frame at time.

    .. method::  canTransformFull(target_frame, target_time, source_frame, source_time, fixed_frame) -> bool

        Extended version of :meth:`canTransform`.

        :param target_time: The time at which to transform into the target_frame.
        :param source_time: The time at which to transform from the source frame.
        :param fixed_frame: The frame in which to jump from source_time to target_time.

    .. method::  clear() -> None

        Clear all transformations from the buffer.

    .. method::  lookupTransform(target_frame, source_frame, time) -> geometry_msgs.msg.TransformStamped

        :param target_frame: transformation target frame in tf, string
        :param source_frame: transformation source frame in tf, string
        :param time: time of the transformation, use ``rospy.Time()`` to indicate most recent common time.
        :returns: position as a translation (x, y, z) and orientation as a quaternion (x, y, z, w)
        :raises: :exc:`tf2.ConnectivityException`, :exc:`tf2.LookupException`, or :exc:`tf2.ExtrapolationException`, or :exc:`tf2.InvalidArgumentException`

        Returns the transform from source_frame to target_frame at time. Raises one of the exceptions if the transformation is not possible.

        Note that a time of zero means latest common time, so::

            t.lookupTransform("a", "b", rospy.Time())

        will return the transform from "a" to "b" at the latest time available in all transforms between "a" and "b".


    .. method::  lookupTransformFull(target_frame, target_time, source_frame, source_time, fixed_frame) -> geometry_msgs.msg.TransformStamped

        :param target_frame: transformation target frame in tf, string
        :param target_time: time of transformation in target_frame, a :class:`rospy.Time`
        :param source_frame: transformation source frame in tf, string
        :param source_time: time of transformation in target_frame, a :class:`rospy.Time`
        :param fixed_frame: reference frame common to both target_frame and source_frame.
        :raises: :exc:`tf2.ConnectivityException`, :exc:`tf2.LookupException`, or :exc:`tf2.ExtrapolationException`, or :exc:`tf2.InvalidArgumentException`

        Extended version of :meth:`lookupTransform`.
