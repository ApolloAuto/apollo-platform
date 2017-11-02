# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# -*- coding: utf-8 -*-

""" diagnostic_updater for Python.
@author Brice Rebsamen <brice [dot] rebsamen [gmail]>
"""

import roslib
roslib.load_manifest('diagnostic_updater')
import rospy
from ._diagnostic_updater import *


class FrequencyStatusParam:
    """A structure that holds the constructor parameters for the FrequencyStatus
    class.

    Implementation note: the min_freq and max_freq parameters in the C += 1 code
    are stored as pointers, so that if they are updated, the new values are used.
    To emulate this behavior, we here use a dictionary to hold them: {'min','max'}

    freq_bound is a dictionary with keys 'min' and 'max', containing the min
    and max acceptable frequencies.

    tolerance is the tolerance with which bounds must be satisfied. Acceptable
    values are from freq_bound['min'] * (1 - torelance) to
    freq_bound['max'] * (1 + tolerance). Common use cases are to set
    tolerance to zero, or to assign the same value to freq_bound['min'] and
    freq_bound['max']

    window_size is the number of events to consider in the statistics.
    """

    def __init__(self, freq_bound, tolerance = 0.1, window_size = 5):
        """Creates a filled-out FrequencyStatusParam."""
        self.freq_bound = freq_bound
        self.tolerance = tolerance
        self.window_size = window_size


class FrequencyStatus(DiagnosticTask):
    """A diagnostic task that monitors the frequency of an event.

    This diagnostic task monitors the frequency of calls to its tick method,
    and creates corresponding diagnostics. It will report a warning if the
    frequency is outside acceptable bounds, and report an error if there have
    been no events in the latest window.
    """

    def __init__(self, params):
        """Constructs a FrequencyStatus class with the given parameters."""
        DiagnosticTask.__init__(self, "Frequency Status")
        self.params = params
        self.lock = threading.Lock()
        self.clear()

    def clear(self):
        """Resets the statistics."""
        with self.lock:
            self.count = 0
            curtime = rospy.Time.now()
            self.times = [curtime for i in range(self.params.window_size)]
            self.seq_nums = [0 for i in range(self.params.window_size)]
            self.hist_indx = 0

    def tick(self):
        """Signals that an event has occurred."""
        with self.lock:
            self.count += 1

    def run(self, stat):
        with self.lock:
            curtime = rospy.Time.now()
            curseq = self.count
            events = curseq - self.seq_nums[self.hist_indx]
            window = (curtime - self.times[self.hist_indx]).to_sec()
            freq = events / window
            self.seq_nums[self.hist_indx] = curseq
            self.times[self.hist_indx] = curtime
            self.hist_indx = (self.hist_indx + 1) % self.params.window_size

            if events == 0:
                stat.summary(2, "No events recorded.")
            elif freq < self.params.freq_bound['min'] * (1 - self.params.tolerance):
                stat.summary(1, "Frequency too low.")
            elif self.params.freq_bound.has_key('max') and freq > self.params.freq_bound['max'] * (1 + self.params.tolerance):
                stat.summary(1, "Frequency too high.")
            else:
                stat.summary(0, "Desired frequency met")

            stat.add("Events in window", "%d" % events)
            stat.add("Events since startup", "%d" % self.count)
            stat.add("Duration of window (s)", "%f" % window)
            stat.add("Actual frequency (Hz)", "%f" % freq)
            if self.params.freq_bound.has_key('max') and self.params.freq_bound['min'] == self.params.freq_bound['max']:
                stat.add("Target frequency (Hz)", "%f" % self.params.freq_bound['min'])
            if self.params.freq_bound['min'] > 0:
                stat.add("Minimum acceptable frequency (Hz)", "%f" % (self.params.freq_bound['min'] * (1 - self.params.tolerance)))
            if self.params.freq_bound.has_key('max'):
                stat.add("Maximum acceptable frequency (Hz)", "%f" % (self.params.freq_bound['max'] * (1 + self.params.tolerance)))

        return stat


class TimeStampStatusParam:
    """A structure that holds the constructor parameters for the TimeStampStatus class.

    max_acceptable: maximum acceptable difference between two timestamps.
    min_acceptable: minimum acceptable difference between two timestamps.
    """

    def __init__(self, min_acceptable = -1, max_acceptable = 5):
        """Creates a filled-out TimeStampStatusParam."""
        self.max_acceptable = max_acceptable
        self.min_acceptable = min_acceptable


class TimeStampStatus(DiagnosticTask):
    """Diagnostic task to monitor the interval between events.

    This diagnostic task monitors the difference between consecutive events,
    and creates corresponding diagnostics. An error occurs if the interval
    between consecutive events is too large or too small. An error condition
    will only be reported during a single diagnostic report unless it
    persists. Tallies of errors are also maintained to keep track of errors
    in a more persistent way.
    """

    def __init__(self, params = TimeStampStatusParam()):
        """Constructs the TimeStampStatus with the given parameters."""
        DiagnosticTask.__init__(self, "Timestamp Status")
        self.params = params
        self.lock = threading.Lock()
        self.early_count = 0
        self.late_count = 0
        self.zero_count = 0
        self.zero_seen = False
        self.max_delta = 0
        self.min_delta = 0
        self.deltas_valid = False

    def tick(self, stamp):
        """Signals an event.
        @param stamp The timestamp of the event that will be used in computing
        intervals. Can be either a double or a ros::Time.
        """
        if not isinstance(stamp, float):
            stamp = stamp.to_sec()

        with self.lock:
            if stamp == 0:
                self.zero_seen = True
            else:
                delta = rospy.Time.now().to_sec() - stamp
                if not self.deltas_valid or delta > self.max_delta:
                    self.max_delta = delta
                if not self.deltas_valid or delta < self.min_delta:
                    self.min_delta = delta
                self.deltas_valid = True

    def run(self, stat):
        with self.lock:

            stat.summary(0, "Timestamps are reasonable.")
            if not self.deltas_valid:
                stat.summary(1, "No data since last update.")
            else:
                if self.min_delta < self.params.min_acceptable:
                    stat.summary(2, "Timestamps too far in future seen.")
                    self.early_count += 1
                if self.max_delta > self.params.max_acceptable:
                    stat.summary(2, "Timestamps too far in past seen.")
                    self.late_count += 1
                if self.zero_seen:
                    stat.summary(2, "Zero timestamp seen.")
                    self.zero_count += 1

            stat.add("Earliest timestamp delay:", "%f" % self.min_delta)
            stat.add("Latest timestamp delay:", "%f" % self.max_delta)
            stat.add("Earliest acceptable timestamp delay:", "%f" % self.params.min_acceptable)
            stat.add("Latest acceptable timestamp delay:", "%f" % self.params.max_acceptable)
            stat.add("Late diagnostic update count:", "%i" % self.late_count)
            stat.add("Early diagnostic update count:", "%i" % self.early_count)
            stat.add("Zero seen diagnostic update count:", "%i" % self.zero_count)

            self.deltas_valid = False
            self.min_delta = 0
            self.max_delta = 0
            self.zero_seen = False

        return stat