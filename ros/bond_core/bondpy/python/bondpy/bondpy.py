# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

## \author Stuart Glaser

import sys
import threading
import time
import uuid

import rospy
from bond.msg import *

import BondSM_sm

def as_ros_duration(d):
    if not isinstance(d, rospy.Duration):
        return rospy.Duration.from_sec(d)
    return d
def as_float_duration(d):
    if isinstance(d, rospy.Duration):
        return d.to_sec()
    return d


## Internal use only
class Timeout:
    def __init__(self, duration, on_timeout=None):
        self.duration = duration
        self.timer = threading.Timer(0, self._on_timer)
        self.deadline = time.time()
        self.on_timeout = on_timeout

    def reset(self):
        self.timer.cancel()
        self.timer = threading.Timer(self.duration.to_sec(), self._on_timer)
        self.timer.start()
        self.deadline = time.time() + self.duration.to_sec()
        return self

    def cancel(self):
        self.timer.cancel()

    def left(self):
        return max(rospy.Duration(0), rospy.Duration(self.deadline - time.time()))

    def _on_timer(self):
        if self.on_timeout:
            self.on_timeout()

## \brief Forms a bond to monitor another process.
#
# The Bond class implements a bond, allowing you to monitor another
# process and be notified when it dies.  In turn, it will be notified
# when you die.
class Bond(object):
    ## \brief Constructs a bond, but does not connect.
    #
    # \param topic The topic used to exchange the bond status messages.
    # \param id The ID of the bond, which should match the ID used on
    #           the sister's end
    # \param on_broken callback that will be called when the bond is broken.
    # \param on_formed callback that will be called when the bond is formed.
    def __init__(self, topic, id, on_broken=None, on_formed=None):
        self.__started = False
        self.topic = topic
        self.id = id
        self.instance_id = str(uuid.uuid4())
        self.sister_instance_id = None
        self.on_broken = on_broken
        self.on_formed = on_formed
        self.is_shutdown = False
        self.sister_died_first = False
        # Timeout for wait_until_formed
        self.deadline = None

        # Callbacks must be called outside of the locks and after the
        # state machine finishes transitioning.
        self.pending_callbacks = []

        self.sm = BondSM_sm.BondSM_sm(self)
        #self.sm.setDebugFlag(True)
        self.lock = threading.RLock()
        self.condition = threading.Condition(self.lock)

        # Sets the default timeout values
        self.connect_timeout = Constants.DEFAULT_CONNECT_TIMEOUT
        self.heartbeat_timeout = Constants.DEFAULT_HEARTBEAT_TIMEOUT
        self.disconnect_timeout = Constants.DEFAULT_DISCONNECT_TIMEOUT
        self.heartbeat_period = Constants.DEFAULT_HEARTBEAT_PERIOD

        # queue_size 1 : avoid having a client receive backed up, potentially
        # late heartbearts, discussion@https://github.com/ros/bond_core/pull/10
        self.pub = rospy.Publisher(self.topic, Status, queue_size=1)

    def get_connect_timeout(self):
        return self.__connect_timeout
    def set_connect_timeout(self, dur):
        assert not self.__started
        self.__connect_timeout = dur
        self.connect_timer = Timeout(as_ros_duration(dur), self._on_connect_timeout)
    connect_timeout = property(get_connect_timeout, set_connect_timeout)

    def get_heartbeat_timeout(self):
        return self.__heartbeat_timeout
    def set_heartbeat_timeout(self, dur):
        assert not self.__started
        self.__heartbeat_timeout = dur
        self.heartbeat_timer = Timeout(as_ros_duration(dur), self._on_heartbeat_timeout)
    heartbeat_timeout = property(get_heartbeat_timeout, set_heartbeat_timeout)

    def get_disconnect_timeout(self):
        return self.__disconnect_timeout
    def set_disconnect_timeout(self, dur):
        assert not self.__started
        self.__disconnect_timeout = dur
        self.disconnect_timer = Timeout(as_ros_duration(dur), self._on_disconnect_timeout)
    disconnect_timeout = property(get_disconnect_timeout, set_disconnect_timeout)

    def get_heartbeat_period(self):
        return self.__heartbeat_period
    def set_heartbeat_period(self, per):
        assert not self.__started
        self.__heartbeat_period = as_float_duration(per)
    heartbeat_period = property(get_heartbeat_period, set_heartbeat_period)


    ## \brief Starts the bond and connects to the sister process.
    def start(self):
        with self.lock:
            self.connect_timer.reset()
            self.sub = rospy.Subscriber(self.topic, Status, self._on_bond_status)

            self.thread = threading.Thread(target=self._publishing_thread)
            self.thread.daemon = True
            self.thread.start()
            self.__started = True


    def _on_connect_timeout(self):
        with self.lock:
            self.sm.ConnectTimeout()
        self._flush_pending_callbacks()

    def _on_heartbeat_timeout(self):
        # Checks that heartbeat timeouts haven't been disabled globally
        disable_heartbeat_timeout = rospy.get_param(Constants.DISABLE_HEARTBEAT_TIMEOUT_PARAM, False)
        if disable_heartbeat_timeout:
            rospy.logwarn("Heartbeat timeout is disabled.  Not breaking bond (topic: %s, id: %s)" % \
                              (self.topic, self.id))
            return

        with self.lock:
            self.sm.HeartbeatTimeout()
        self._flush_pending_callbacks()

    def _on_disconnect_timeout(self):
        with self.lock:
            self.sm.DisconnectTimeout()
        self._flush_pending_callbacks()

    def __del__(self):
        self.shutdown()

    def shutdown(self):
        if not self.is_shutdown:
            self.sub.unregister()
            with self.lock:
                self.is_shutdown = True
                if self.sm.getState().getName() != 'SM.Dead':
                    self.break_bond()
                self.pub.unregister()
                self.condition.notify_all()
                self.connect_timer.cancel()
                if self.deadline:
                    self.deadline.cancel()

    def _on_bond_status(self, msg):
        # Filters out messages from other bonds and messages from ourself
        if msg.id == self.id and msg.instance_id != self.instance_id:
            with self.lock:
                if not self.sister_instance_id:
                    self.sister_instance_id = msg.instance_id

                if msg.instance_id != self.sister_instance_id:
                    rospy.logerr("More than two locations are trying to use a single bond (topic: %s, id: %s).  " + \
                                 "You should only instantiate at most two bond instances for each (topic, id) pair." % \
                                     (self.topic, self.id))
                    return

                if msg.active:
                    self.sm.SisterAlive()
                else:
                    self.sm.SisterDead()

                    # Immediate ack for sister's death notification
                    if self.sister_died_first:
                        self._publish(False)
            self._flush_pending_callbacks()



    def _publish(self, active):
        msg = Status()
        msg.header.stamp = rospy.Time.now()
        msg.id = self.id
        msg.instance_id = self.instance_id
        msg.active = active
        msg.heartbeat_timeout = self.heartbeat_timeout
        msg.heartbeat_period = self.heartbeat_period
        self.pub.publish(msg)

    def _publishing_thread(self):
        with self.lock:
            # Publishing ALIVE
            while not self.is_shutdown and self.sm.getState().getName() in ['SM.WaitingForSister', 'SM.Alive']:
                self._publish(True)
                self.condition.wait(self.heartbeat_period)

            # Publishing DEAD
            while not self.is_shutdown and self.sm.getState().getName() == 'SM.AwaitSisterDeath':
                self._publish(False)
                self.condition.wait(Constants.DEAD_PUBLISH_PERIOD)

    def _flush_pending_callbacks(self):
        callbacks = []
        with self.lock:
            callbacks = self.pending_callbacks
            self.pending_callbacks = []
        for c in callbacks:
            c()

    ## \brief INTERNAL
    def Connected(self):
        self.connect_timer.cancel()
        self.condition.notify_all()
        if self.on_formed:
            self.pending_callbacks.append(self.on_formed)

    ## \brief INTERNAL
    def Heartbeat(self):
        self.heartbeat_timer.reset()

    ## \brief INTERNAL
    def SisterDied(self):
        self.sister_died_first = True

    ## \brief INTERNAL
    def Death(self):
        self.condition.notify_all()
        self.heartbeat_timer.cancel()
        self.disconnect_timer.cancel()
        if self.on_broken:
            self.pending_callbacks.append(self.on_broken)

    ## \brief INTERNAL
    def StartDying(self):
        self.heartbeat_timer.cancel()
        self.disconnect_timer.reset()

    ## \brief Sets the formed callback
    def set_formed_callback(self, on_formed):
        with self.lock:
            self.on_formed = on_formed

    ## \brief Sets the broken callback
    def set_broken_callback(self, on_broken):
        with self.lock:
            self.on_broken = on_broken

    ## \brief Blocks until the bond is formed for at most 'duration'.
    #
    # \param timeout Maximum duration to wait.  If None then this call will not timeout.
    # \return true iff the bond has been formed.
    def wait_until_formed(self, timeout = None):
        if self.deadline:
            self.deadline.cancel()
            self.deadline = None
        if timeout:
            self.deadline = Timeout(timeout).reset()
        with self.lock:
            while self.sm.getState().getName() == 'SM.WaitingForSister':
                if rospy.is_shutdown():
                    break
                if self.deadline and self.deadline.left() == rospy.Duration(0):
                    break
                wait_duration = 0.1
                if self.deadline:
                    wait_duration = min(wait_duration, self.deadline.left().to_sec())
                self.condition.wait(wait_duration)
            return self.sm.getState().getName() != 'SM.WaitingForSister'

    ## \brief Blocks until the bond is broken for at most 'duration'.
    #
    # \param timeout Maximum duration to wait.  If None then this call will not timeout.
    # \return true iff the bond has been broken, even if it has never been formed.
    def wait_until_broken(self, timeout = None):
        if self.deadline:
            self.deadline.cancel()
            self.deadline = None
        if timeout:
            self.deadline = Timeout(timeout).reset()
        with self.lock:
            while self.sm.getState().getName() != 'SM.Dead':
                if rospy.is_shutdown():
                    break
                if self.deadline and self.deadline.left() == rospy.Duration(0):
                    break
                wait_duration = 0.1
                if self.deadline:
                    wait_duration = min(wait_duration, self.deadline.left().to_sec())
                self.condition.wait(wait_duration)
            return self.sm.getState().getName() == 'SM.Dead'

    ## \brief Indicates if the bond is broken
    # \return true iff the bond has been broken.
    def is_broken(self):
        with self.lock:
            return self.sm.getState().getName() == 'SM.Dead'

    ## \brief Breaks the bond, notifying the other process.
    def break_bond(self):
        with self.lock:
            self.sm.Die()
            self._publish(False)
        self._flush_pending_callbacks()


    def __repr__(self):
        return "[Bond %s, Instance %s (%s)]" % (self.id, self.instance_id, self.sm.getState().getName())
