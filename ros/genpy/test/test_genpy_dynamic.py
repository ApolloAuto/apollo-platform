# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

try:
    from cStringIO import StringIO
except ImportError:
    from io import BytesIO as StringIO
import sys
import time

def test_generate_dynamic():
    import genpy
    from genpy.dynamic import generate_dynamic
    msgs = generate_dynamic("gd_msgs/EasyString", "string data\n")
    assert ['gd_msgs/EasyString'] == list(msgs.keys())
    m_cls = msgs['gd_msgs/EasyString']
    m_instance = m_cls()
    m_instance.data = 'foo'
    buff = StringIO()
    m_instance.serialize(buff)
    m_instance2 = m_cls().deserialize(buff.getvalue())
    assert m_instance == m_instance2

    try:
        char = unichr
    except NameError:
        char = chr
    m_instance.data = 'foo' + char(1234)
    buff = StringIO()
    m_instance.serialize(buff)
    m_instance2 = m_cls().deserialize(buff.getvalue())
    if sys.hexversion < 0x03000000:
        # python 2 requires manual decode into unicode
        m_instance2.data = m_instance2.data.decode('utf-8')
    assert m_instance == m_instance2

    # 'probot_msgs' is a test for #1183, failure if the package no longer exists
    msgs = generate_dynamic("gd_msgs/MoveArmState", """Header header
probot_msgs/ControllerStatus status

#Current arm configuration
probot_msgs/JointState[] configuration
#Goal arm configuration
probot_msgs/JointState[] goal

================================================================================
MSG: std_msgs/Header
#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: probot_msgs/ControllerStatus
# This message defines the expected format for Controller Statuss messages
# Embed this in the feedback state message of highlevel controllers
byte UNDEFINED=0
byte SUCCESS=1
byte ABORTED=2
byte PREEMPTED=3
byte ACTIVE=4

# Status of the controller = {UNDEFINED, SUCCESS, ABORTED, PREEMPTED, ACTIVE}
byte value

#Comment for debug
string comment
================================================================================
MSG: probot_msgs/JointState
string name
float64 position
float64 velocity
float64 applied_effort
float64 commanded_effort
byte is_calibrated

""")
    assert set(['gd_msgs/MoveArmState', 'probot_msgs/JointState', 'probot_msgs/ControllerStatus', 'std_msgs/Header']) ==  set(msgs.keys())
    m_instance1 = msgs['std_msgs/Header']() # make sure default constructor works
    m_instance2 = msgs['std_msgs/Header'](stamp=genpy.Time.from_sec(time.time()), frame_id='foo-%s'%time.time(), seq=12390)
    _test_ser_deser(m_instance2, m_instance1)

    m_instance1 = msgs['probot_msgs/ControllerStatus']()
    m_instance2 = msgs['probot_msgs/ControllerStatus'](value=4, comment=str(time.time()))
    d = {'UNDEFINED':0,'SUCCESS':1,'ABORTED':2,'PREEMPTED':3,'ACTIVE':4}
    for k, v in d.items():
        assert v == getattr(m_instance1, k)
    _test_ser_deser(m_instance2, m_instance1)

    m_instance1 = msgs['probot_msgs/JointState']()
    m_instance2 = msgs['probot_msgs/JointState'](position=time.time(), velocity=time.time(), applied_effort=time.time(), commanded_effort=time.time(), is_calibrated=2)
    _test_ser_deser(m_instance2, m_instance1)

    m_instance1 = msgs['gd_msgs/MoveArmState']()
    js = msgs['probot_msgs/JointState']
    config = []
    goal = []
    # generate some data for config/goal
    for i in range(0, 10):
        config.append(js(position=time.time(), velocity=time.time(), applied_effort=time.time(), commanded_effort=time.time(), is_calibrated=2))
        goal.append(js(position=time.time(), velocity=time.time(), applied_effort=time.time(), commanded_effort=time.time(), is_calibrated=2))
    m_instance2 = msgs['gd_msgs/MoveArmState'](header=msgs['std_msgs/Header'](),
                                               status=msgs['probot_msgs/ControllerStatus'](),
                                               configuration=config, goal=goal)
    _test_ser_deser(m_instance2, m_instance1)

def _test_ser_deser(m_instance1, m_instance2):
    buff = StringIO()
    m_instance1.serialize(buff)
    m_instance2.deserialize(buff.getvalue())
    assert m_instance1 == m_instance2


def test_serialize_exception():
    import genpy
    from genpy.dynamic import generate_dynamic
    msgs = generate_dynamic("gd_msgs/EasyInt32", "int32 data\n")
    assert ['gd_msgs/EasyInt32'] == list(msgs.keys())
    m_cls = msgs['gd_msgs/EasyInt32']
    m_instance = m_cls()
    m_instance.data = '1'  # the type is incorrect
    buff = StringIO()
    try:
        m_instance.serialize(buff)
        assert False, 'This should have raised a genpy.SerializationError'
    except genpy.SerializationError:
        pass
    except Exception:
        assert False, 'This should have raised a genpy.SerializationError instead'
