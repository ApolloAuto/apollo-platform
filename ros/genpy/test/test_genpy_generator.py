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

import os

import genmsg.msgs
from genmsg.msgs import MsgSpec
from genmsg.msg_loader import MsgContext

def get_test_dir():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), 'files'))

def test_is_special():
    import genpy.generator
    for t in ['time', 'duration', 'Header']:
        assert genpy.generator.is_special(t)
        
def test_Simple():
    import genpy.generator
    val = genpy.generator.get_special('time').import_str
    assert 'import genpy' == val, val
    assert 'import genpy' == genpy.generator.get_special('duration').import_str
    assert 'import std_msgs.msg' == genpy.generator.get_special('Header').import_str

    assert 'genpy.Time()' == genpy.generator.get_special('time').constructor
    assert 'genpy.Duration()' == genpy.generator.get_special('duration').constructor
    assert 'std_msgs.msg._Header.Header()' == genpy.generator.get_special('Header').constructor

    assert 'self.foo.canon()' == genpy.generator.get_special('time').get_post_deserialize('self.foo')
    assert 'bar.canon()' == genpy.generator.get_special('time').get_post_deserialize('bar')
    assert 'self.foo.canon()' == genpy.generator.get_special('duration').get_post_deserialize('self.foo')
    assert None == genpy.generator.get_special('Header').get_post_deserialize('self.foo')

def test_compute_post_deserialize():
    import genpy.generator
    assert 'self.bar.canon()' == genpy.generator.compute_post_deserialize('time', 'self.bar')
    assert 'self.bar.canon()' == genpy.generator.compute_post_deserialize('duration', 'self.bar')
    assert None == genpy.generator.compute_post_deserialize('Header', 'self.bar')

    assert None == genpy.generator.compute_post_deserialize('int8', 'self.bar')
    assert None == genpy.generator.compute_post_deserialize('string', 'self.bar')

def test_flatten():
    import genpy.generator
    from genpy.generator import flatten
    msg_context = MsgContext.create_default()

    simple = MsgSpec(['string'], ['data'], [], 'string data\n', 'simple/String')
    simple2 = MsgSpec(['string', 'int32'], ['data', 'data2'], [], 'string data\nint32 data2\n', 'simpe/Data2')
    assert simple == flatten(msg_context, simple)
    assert simple2 == flatten(msg_context, simple2)

    b1 = MsgSpec(['int8'], ['data'], [], 'X', 'f_msgs/Base')
    b2 = MsgSpec(['f_msgs/Base'], ['data'], [], 'X', 'f_msgs/Base2')
    b3 = MsgSpec(['f_msgs/Base2', 'f_msgs/Base2'], ['data3', 'data4'], [], 'X', 'f_msgs/Base3')
    b4 = MsgSpec(['f_msgs/Base3', 'f_msgs/Base3'], ['dataA', 'dataB'], [], 'X', 'f_msgs/Base4')

    msg_context.register('f_msgs/Base', b1)
    msg_context.register('f_msgs/Base2', b2)
    msg_context.register('f_msgs/Base3', b3)
    msg_context.register('f_msgs/Base4', b4)

    assert MsgSpec(['int8'], ['data.data'], [], 'X', 'f_msgs/Base2') == flatten(msg_context, b2)
    assert MsgSpec(['int8', 'int8'], ['data3.data.data', 'data4.data.data'], [], 'X', 'f_msgs/Base3') == flatten(msg_context, b3)
    assert MsgSpec(['int8', 'int8', 'int8', 'int8'],
                              ['dataA.data3.data.data', 'dataA.data4.data.data', 'dataB.data3.data.data', 'dataB.data4.data.data'],
                              [], 'X', 'f_msgs/Base4') == flatten(msg_context, b4)

def test_flatten_array_objects():
    # make sure array of types don't flatten
    from genpy.generator import flatten
    msg_context = MsgContext.create_default()

    b1 = MsgSpec(['int8'], ['data'], [], 'X', 'f_msgs/Base')
    b5 = MsgSpec(['f_msgs/Base[]'], ['data'], [], 'X', 'f_msgs/Base5')

    msg_context.register('f_msgs/Base', b1)
    msg_context.register('f_msgs/Base5', b5)
    assert b5 == flatten(msg_context, b5)
    
def test_default_value():
    from genpy.generator import default_value
    msg_context = MsgContext.create_default()

    msg_context.register('fake_msgs/String', MsgSpec(['string'], ['data'], [], 'string data\n', 'fake_msgs/String'))
    msg_context.register('fake_msgs/ThreeNums', MsgSpec(['int32', 'int32', 'int32'], ['x', 'y', 'z'], [], 'int32 x\nint32 y\nint32 z\n', 'fake_msgs/ThreeNums'))

    # trip-wire: make sure all builtins have a default value
    for t in genmsg.msgs.BUILTIN_TYPES:
        assert type(default_value(msg_context, t, 'roslib')) == str

    # simple types first
    for t in ['uint8', 'int8', 'uint16', 'int16', 'uint32', 'int32', 'uint64', 'int64', 'byte', 'char']:
        assert '0' == default_value(msg_context, t, 'std_msgs')
        assert '0' == default_value(msg_context, t, 'roslib')
    for t in ['float32', 'float64']:
        assert '0.' == default_value(msg_context, t, 'std_msgs')
        assert '0.' == default_value(msg_context, t, 'roslib')
    assert "''" == default_value(msg_context, 'string', 'roslib')

    # builtin specials
    assert 'genpy.Time()' == default_value(msg_context, 'time', 'roslib')
    assert 'genpy.Duration()' == default_value(msg_context, 'duration', 'roslib')
    assert 'std_msgs.msg._Header.Header()' == default_value(msg_context, 'Header', 'roslib')

    assert 'genpy.Time()' == default_value(msg_context, 'time', 'std_msgs')
    assert 'genpy.Duration()' == default_value(msg_context, 'duration', 'std_msgs')
    assert 'std_msgs.msg._Header.Header()' == default_value(msg_context, 'Header', 'std_msgs')

    # generic instances
    # - unregistered type
    assert None == default_value(msg_context, "unknown_msgs/Foo", "unknown_msgs")
    # - wrong context
    assert None == default_value(msg_context, 'ThreeNums', 'std_msgs')

    # - registered types
    assert 'fake_msgs.msg.String()' == default_value(msg_context, 'fake_msgs/String', 'std_msgs')
    assert 'fake_msgs.msg.String()' == default_value(msg_context, 'fake_msgs/String', 'fake_msgs')
    assert 'fake_msgs.msg.String()' == default_value(msg_context, 'String', 'fake_msgs')
    assert 'fake_msgs.msg.ThreeNums()' == default_value(msg_context, 'fake_msgs/ThreeNums', 'roslib')
    assert 'fake_msgs.msg.ThreeNums()' == default_value(msg_context, 'fake_msgs/ThreeNums', 'fake_msgs')
    assert 'fake_msgs.msg.ThreeNums()' == default_value(msg_context, 'ThreeNums', 'fake_msgs')

    # var-length arrays always default to empty arrays... except for byte and uint8 which are strings
    for t in ['int8', 'uint16', 'int16', 'uint32', 'int32', 'uint64', 'int64', 'float32', 'float64']:
        val = default_value(msg_context, t+'[]', 'std_msgs')
        assert '[]' == val, "[%s]: %s"%(t, val)
        assert '[]' == default_value(msg_context, t+'[]', 'roslib')

    assert "''" == default_value(msg_context, 'uint8[]', 'roslib')

    # fixed-length arrays should be zero-filled... except for byte and uint8 which are strings
    for t in ['float32', 'float64']:
        assert '[0.,0.,0.]' == default_value(msg_context, t+'[3]', 'std_msgs')
        assert '[0.]' == default_value(msg_context, t+'[1]', 'std_msgs')
    for t in ['int8', 'uint16', 'int16', 'uint32', 'int32', 'uint64', 'int64']:
        assert '[0,0,0,0]' == default_value(msg_context, t+'[4]', 'std_msgs')
        assert '[0]' == default_value(msg_context, t+'[1]', 'roslib')

    assert "chr(0)*1" == default_value(msg_context, 'uint8[1]', 'roslib')
    assert "chr(0)*4" == default_value(msg_context, 'uint8[4]', 'roslib')

    assert '[]' == default_value(msg_context, 'fake_msgs/String[]', 'std_msgs')
    assert '[fake_msgs.msg.String(),fake_msgs.msg.String()]' == default_value(msg_context, 'fake_msgs/String[2]', 'std_msgs')

def test_make_python_safe():
    from genpy.generator import make_python_safe
    from genmsg.msgs import Constant
    s = MsgSpec(['int32', 'int32', 'int32', 'int32', 'int32', 'int32'], ['ok', 'if', 'self', 'fine', 'self.x', 'self.while'],
                [Constant('int32', 'if', '1', '1'), Constant('int32', 'okgo', '1', '1')],
                'x', 'test_msgs/Foo')
    s2 = make_python_safe(s)
    assert s != s2
    assert ['ok', 'if_', 'self_', 'fine', 'self.x', 'self.while_'] == s2.names, s2.names
    assert s2.types == s.types
    assert [Constant('int32', 'if_', '1', '1') == Constant('int32', 'okgo', '1', '1')], s2.constants
    assert s2.text == s.text
    
def test_compute_pkg_type():
    from genpy.generator import compute_pkg_type, MsgGenerationException
    try:
        compute_pkg_type('std_msgs', 'really/bad/std_msgs/String')
    except MsgGenerationException: pass
    assert ('std_msgs', 'String') == compute_pkg_type('std_msgs', 'std_msgs/String')
    assert ('std_msgs', 'String') == compute_pkg_type('foo', 'std_msgs/String')    
    assert ('std_msgs', 'String') == compute_pkg_type('std_msgs', 'String')
        
def test_compute_import():
    import genpy.generator
    msg_context = MsgContext.create_default()

    assert [] == genpy.generator.compute_import(msg_context, 'foo', 'bar')
    assert [] == genpy.generator.compute_import(msg_context, 'foo', 'int32')

    msg_context.register('ci_msgs/Base', MsgSpec(['int8'], ['data'], [], 'int8 data\n', 'ci_msgs/Base'))
    msg_context.register('ci2_msgs/Base2', MsgSpec(['ci_msgs/Base'], ['data2'], [], 'ci_msgs/Base data2\n', 'ci2_msgs/Base2'))
    msg_context.register('ci3_msgs/Base3', MsgSpec(['ci2_msgs/Base2'], ['data3'], [], 'ci2_msgs/Base2 data3\n', 'ci3_msgs/Base3'))
    msg_context.register('ci4_msgs/Base', MsgSpec(['int8'], ['data'], [], 'int8 data\n', 'ci4_msgs/Base'))
    msg_context.register('ci4_msgs/Base4', MsgSpec(['ci2_msgs/Base2', 'ci3_msgs/Base3'],
                                       ['data4a', 'data4b'],
                                       [], 'ci2_msgs/Base2 data4a\nci3_msgs/Base3 data4b\n', 'ci4_msgs/Base4'))

    msg_context.register('ci5_msgs/Base', MsgSpec(['time'], ['data'], [], 'time data\n', 'ci5_msgs/Base'))

    assert ['import ci_msgs.msg'] == genpy.generator.compute_import(msg_context, 'foo', 'ci_msgs/Base')
    assert ['import ci_msgs.msg'] == genpy.generator.compute_import(msg_context, 'ci_msgs', 'ci_msgs/Base')
    assert ['import ci2_msgs.msg', 'import ci_msgs.msg'] == genpy.generator.compute_import(msg_context, 'ci2_msgs', 'ci2_msgs/Base2')
    assert ['import ci2_msgs.msg', 'import ci_msgs.msg'] == genpy.generator.compute_import(msg_context, 'foo', 'ci2_msgs/Base2')
    assert ['import ci3_msgs.msg', 'import ci2_msgs.msg', 'import ci_msgs.msg'] == genpy.generator.compute_import(msg_context, 'ci3_msgs', 'ci3_msgs/Base3')

    assert set(['import ci4_msgs.msg', 'import ci3_msgs.msg', 'import ci2_msgs.msg', 'import ci_msgs.msg']) == set(genpy.generator.compute_import(msg_context, 'foo', 'ci4_msgs/Base4'))
    assert set(['import ci4_msgs.msg', 'import ci3_msgs.msg', 'import ci2_msgs.msg', 'import ci_msgs.msg']) == set(genpy.generator.compute_import(msg_context, 'ci4_msgs', 'ci4_msgs/Base4'))

    assert ['import ci4_msgs.msg'] == genpy.generator.compute_import(msg_context, 'foo', 'ci4_msgs/Base')    
    assert ['import ci4_msgs.msg'] == genpy.generator.compute_import(msg_context, 'ci4_msgs', 'ci4_msgs/Base')
    assert ['import ci4_msgs.msg'] == genpy.generator.compute_import(msg_context, 'ci4_msgs', 'Base')

    assert ['import ci5_msgs.msg', 'import genpy'] == genpy.generator.compute_import(msg_context, 'foo', 'ci5_msgs/Base')
        
def test_get_registered_ex():
    import genpy.generator
    msg_context = MsgContext.create_default()
    s = MsgSpec(['string'], ['data'], [], 'string data\n', 'tgr_msgs/String')
    msg_context.register('tgr_msgs/String', s)
    assert s == genpy.generator.get_registered_ex(msg_context, 'tgr_msgs/String')
    try:
        genpy.generator.get_registered_ex(msg_context, 'bad_msgs/String')
    except genpy.generator.MsgGenerationException: pass
            
def test_compute_constructor():
    from genpy.generator import compute_constructor
    msg_context = MsgContext.create_default()
    msg_context.register('fake_msgs/String', MsgSpec(['string'], ['data'], [], 'string data\n', 'fake_msgs/String'))
    msg_context.register('fake_msgs/ThreeNums', MsgSpec(['int32', 'int32', 'int32'], ['x', 'y', 'z'], [], 'int32 x\nint32 y\nint32 z\n', 'fake_msgs/ThreeNums'))

    # builtin specials
    assert 'genpy.Time()' == compute_constructor(msg_context, 'roslib', 'time')
    assert 'genpy.Duration()' == compute_constructor(msg_context, 'roslib', 'duration')
    assert 'std_msgs.msg._Header.Header()' == compute_constructor(msg_context, 'std_msgs', 'Header')

    assert 'genpy.Time()' == compute_constructor(msg_context, 'std_msgs', 'time')
    assert 'genpy.Duration()' == compute_constructor(msg_context, 'std_msgs', 'duration')

    # generic instances
    # - unregistered type
    assert None == compute_constructor(msg_context, "unknown_msgs", "unknown_msgs/Foo")
    assert None == compute_constructor(msg_context, "unknown_msgs", "Foo")
    # - wrong context
    assert None == compute_constructor(msg_context, 'std_msgs', 'ThreeNums')

    # - registered types
    assert 'fake_msgs.msg.String()' == compute_constructor(msg_context, 'std_msgs', 'fake_msgs/String')
    assert 'fake_msgs.msg.String()' == compute_constructor(msg_context, 'fake_msgs', 'fake_msgs/String')
    assert 'fake_msgs.msg.String()' == compute_constructor(msg_context, 'fake_msgs', 'String')
    assert 'fake_msgs.msg.ThreeNums()' == compute_constructor(msg_context, 'fake_msgs', 'fake_msgs/ThreeNums')
    assert 'fake_msgs.msg.ThreeNums()' == compute_constructor(msg_context, 'fake_msgs', 'fake_msgs/ThreeNums')
    assert 'fake_msgs.msg.ThreeNums()' == compute_constructor(msg_context, 'fake_msgs', 'ThreeNums')

def test_len_serializer_generator():
    import genpy.generator
    # generator tests are mainly tripwires/coverage tests
    # Test Serializers
    # string serializer simply initializes local var
    g = genpy.generator.len_serializer_generator('foo', True, True)
    assert 'length = len(foo)' == '\n'.join(g)
    # array len serializer writes var
    g = genpy.generator.len_serializer_generator('foo', False, True)        
    assert "length = len(foo)\nbuff.write(_struct_I.pack(length))" == '\n'.join(g)

    # Test Deserializers
    val = """start = end
end += 4
(length,) = _struct_I.unpack(str[start:end])"""
    # string serializer and array serializer are identical
    g = genpy.generator.len_serializer_generator('foo', True, False)
    assert val == '\n'.join(g)
    g = genpy.generator.len_serializer_generator('foo', False, False)        
    assert val == '\n'.join(g)

def test_string_serializer_generator():
    import genpy.generator
    # generator tests are mainly tripwires/coverage tests
    # Test Serializers
    g = genpy.generator.string_serializer_generator('foo', 'string', 'var_name', True)
    val = '\n'.join(g)
    assert """length = len(var_name)
if python3 or type(var_name) == unicode:
  var_name = var_name.encode('utf-8')
  length = len(var_name)
if python3:
  buff.write(struct.pack('<I%sB'%length, length, *var_name))
else:
  buff.write(struct.pack('<I%ss'%length, length, var_name))""" == val, val

    for t in ['uint8[]', 'byte[]', 'uint8[10]', 'byte[20]']:
        g = genpy.generator.string_serializer_generator('foo', 'uint8[]', 'b_name', True)
        assert """length = len(b_name)
# - if encoded as a list instead, serialize as bytes instead of string
if type(b_name) in [list, tuple]:
  buff.write(struct.pack('<I%sB'%length, length, *b_name))
else:
  buff.write(struct.pack('<I%ss'%length, length, b_name))""" == '\n'.join(g)

    # Test Deserializers
    val = """start = end
end += 4
(length,) = _struct_I.unpack(str[start:end])
start = end
end += length
if python3:
  var_name = str[start:end].decode('utf-8')
else:
  var_name = str[start:end]"""
    # string serializer and array serializer are identical
    g = genpy.generator.string_serializer_generator('foo', 'string', 'var_name', False)
    assert val == '\n'.join(g)


def test_array_serializer_generator_numpy():
    is_numpy = True
    from genpy.generator import array_serializer_generator
    d = os.path.join(get_test_dir(), 'array')
    # generator tests are mainly tripwires/coverage tests

    #array_serializer_generator(msg_context, package, type_, name, serialize, is_numpy):
    msg_context = MsgContext.create_default()

    # permutations: var length, unint8
    serialize = True
    result = array_serializer_generator(msg_context, '', 'uint8[]', 'data', serialize, is_numpy)
    compare_file(d, 'uint8_varlen_ser_np.txt', result)
    result = array_serializer_generator(msg_context, '', 'int16[]', 'data', serialize, is_numpy)
    compare_file(d, 'int16_varlen_ser_np.txt', result)
    result = array_serializer_generator(msg_context, '', 'uint8[8]', 'data', serialize, is_numpy)
    compare_file(d, 'uint8_fixed_ser_np.txt', result)
    result = array_serializer_generator(msg_context, '', 'int16[10]', 'data', serialize, is_numpy)
    compare_file(d, 'int16_fixed_ser_np.txt', result)
    
    serialize = False
    result = array_serializer_generator(msg_context, '', 'uint8[]', 'data', serialize, is_numpy)
    compare_file(d, 'uint8_varlen_deser_np.txt', result)
    result = array_serializer_generator(msg_context, '', 'int16[]', 'data', serialize, is_numpy)
    compare_file(d, 'int16_varlen_deser_np.txt', result)
    result = array_serializer_generator(msg_context, '', 'uint8[8]', 'data', serialize, is_numpy)
    compare_file(d, 'uint8_fixed_deser_np.txt', result)
    result = array_serializer_generator(msg_context, '', 'int16[10]', 'data', serialize, is_numpy)
    compare_file(d, 'int16_fixed_deser_np.txt', result)

def compare_file(d, filename, result):
    result = '\n'.join([l for l in result])
    expected = open(os.path.join(d, filename)).read().strip()
    assert result == expected, "\n[%s]\n[%s]"%(result, expected)
    
def exhaust(gen):
    [g for g in gen]

def test_array_serializer_generator():
    from genmsg.msg_loader import load_msg_by_type
    from genpy.generator import array_serializer_generator, MsgGenerationException, reset_var
    d = os.path.join(get_test_dir(), 'array')
    # generator tests are mainly tripwires/coverage tests

    #array_serializer_generator(msg_context, package, type_, name, serialize, is_numpy):
    msg_context = MsgContext.create_default()
    # load in some objects
    search_path = {'foo': [d]}
    load_msg_by_type(msg_context, 'foo/Object', search_path)
    load_msg_by_type(msg_context, 'foo/ObjectArray', search_path)

    # permutations: var length, unint8
    is_numpy = False

    serialize = True
    result = array_serializer_generator(msg_context, '', 'uint8[]', 'data', serialize, is_numpy)
    compare_file(d, 'uint8_varlen_ser.txt', result)
    result = array_serializer_generator(msg_context, '', 'int16[]', 'data', serialize, is_numpy)
    compare_file(d, 'int16_varlen_ser.txt', result)
    result = array_serializer_generator(msg_context, '', 'uint8[8]', 'data', serialize, is_numpy)
    compare_file(d, 'uint8_fixed_ser.txt', result)
    result = array_serializer_generator(msg_context, '', 'int16[10]', 'data', serialize, is_numpy)
    compare_file(d, 'int16_fixed_ser.txt', result)
    result = array_serializer_generator(msg_context, '', 'bool[]', 'data', serialize, is_numpy)
    compare_file(d, 'bool_varlen_ser.txt', result)
    result = array_serializer_generator(msg_context, '', 'bool[3]', 'data', serialize, is_numpy)
    compare_file(d, 'bool_fixed_ser.txt', result)

    # for 'complex' types have to reset the variable generator
    reset_var()
    result = array_serializer_generator(msg_context, '', 'string[]', 'data', serialize, is_numpy)
    compare_file(d, 'string_varlen_ser.txt', result)
    reset_var()
    result = array_serializer_generator(msg_context, '', 'string[2]', 'data', serialize, is_numpy)
    compare_file(d, 'string_fixed_ser.txt', result)
    
    reset_var()
    result = array_serializer_generator(msg_context, 'foo', 'foo/Object[]', 'data', serialize, is_numpy)
    compare_file(d, 'object_varlen_ser.txt', result)
    reset_var()
    result = array_serializer_generator(msg_context, 'foo', 'foo/Object[3]', 'data', serialize, is_numpy)
    compare_file(d, 'object_fixed_ser.txt', result)

    serialize = False
    result = array_serializer_generator(msg_context, '', 'uint8[]', 'data', serialize, is_numpy)
    compare_file(d, 'uint8_varlen_deser.txt', result)
    result = array_serializer_generator(msg_context, '', 'int16[]', 'data', serialize, is_numpy)
    compare_file(d, 'int16_varlen_deser.txt', result)
    result = array_serializer_generator(msg_context, '', 'uint8[8]', 'data', serialize, is_numpy)
    compare_file(d, 'uint8_fixed_deser.txt', result)
    result = array_serializer_generator(msg_context, '', 'int16[10]', 'data', serialize, is_numpy)
    compare_file(d, 'int16_fixed_deser.txt', result)
    result = array_serializer_generator(msg_context, '', 'bool[]', 'data', serialize, is_numpy)
    compare_file(d, 'bool_varlen_deser.txt', result)
    result = array_serializer_generator(msg_context, '', 'bool[3]', 'data', serialize, is_numpy)
    compare_file(d, 'bool_fixed_deser.txt', result)

    # for 'complex' types have to reset the variable generator
    reset_var()
    result = array_serializer_generator(msg_context, '', 'string[]', 'data', serialize, is_numpy)
    compare_file(d, 'string_varlen_deser.txt', result)
    reset_var()    
    result = array_serializer_generator(msg_context, '', 'string[2]', 'data', serialize, is_numpy)
    compare_file(d, 'string_fixed_deser.txt', result)

    reset_var()
    result = array_serializer_generator(msg_context, 'foo', 'foo/Object[]', 'data', serialize, is_numpy)
    compare_file(d, 'object_varlen_deser.txt', result)
    reset_var()
    result = array_serializer_generator(msg_context, 'foo', 'foo/Object[3]', 'data', serialize, is_numpy)
    compare_file(d, 'object_fixed_deser.txt', result)

    # test w/ bad args
    try:
        result = array_serializer_generator(msg_context, '', 'uint8', 'data', True, False)
        exhaust(result)
        assert False, "should have raised"
    except MsgGenerationException:
        pass

def test_complex_serializer_generator():
    from genmsg.msg_loader import load_msg_by_type
    from genpy.generator import complex_serializer_generator, MsgGenerationException, reset_var
    array_d = os.path.join(get_test_dir(), 'array')
    complex_d = os.path.join(get_test_dir(), 'complex')
    # generator tests are mainly tripwires/coverage tests

    #array_serializer_generator(msg_context, package, type_, name, serialize, is_numpy):
    msg_context = MsgContext.create_default()
    # load in some objects
    search_path = {'foo': [array_d]}
    load_msg_by_type(msg_context, 'foo/Object', search_path)
    load_msg_by_type(msg_context, 'foo/ObjectArray', search_path)


    serialize = True
    is_numpy = False
    reset_var()
    result = complex_serializer_generator(msg_context, 'foo', 'foo/Object', 'data', serialize, is_numpy)
    compare_file(complex_d, 'object_ser.txt', result)
    reset_var()
    result = complex_serializer_generator(msg_context, 'foo', 'foo/Object[]', 'data', serialize, is_numpy)
    compare_file(array_d, 'object_varlen_ser.txt', result)
    reset_var()
    result = complex_serializer_generator(msg_context, 'foo', 'foo/Object[3]', 'data', serialize, is_numpy)
    compare_file(array_d, 'object_fixed_ser.txt', result)

    serialize = False
    
    reset_var()
    result = complex_serializer_generator(msg_context, 'foo', 'foo/Object[]', 'data', serialize, is_numpy)
    compare_file(array_d, 'object_varlen_deser.txt', result)
    reset_var()
    result = complex_serializer_generator(msg_context, 'foo', 'foo/Object[3]', 'data', serialize, is_numpy)
    compare_file(array_d, 'object_fixed_deser.txt', result)

    try:
        result = complex_serializer_generator(msg_context, 'foo', 'bad/Object', 'data', serialize, is_numpy)
        exhaust(result)
        assert False, "should have raised"
    except MsgGenerationException:
        pass


def test_serialize_fn_generator():
    
    from genmsg.msg_loader import load_msg_by_type
    from genpy.generator import serialize_fn_generator, reset_var
    array_d = os.path.join(get_test_dir(), 'array')
    complex_d = os.path.join(get_test_dir(), 'complex')
    # generator tests are mainly tripwires/coverage tests

    #array_serializer_generator(msg_context, package, type_, name, serialize, is_numpy):
    msg_context = MsgContext.create_default()
    # load in some objects
    search_path = {'foo': [array_d]}
    object_spec = load_msg_by_type(msg_context, 'foo/Object', search_path)
    object_array_spec = load_msg_by_type(msg_context, 'foo/ObjectArray', search_path)

    is_numpy = False
    reset_var()
    result = serialize_fn_generator(msg_context, object_spec, is_numpy)
    compare_file(complex_d, 'object_ser_full.txt', result)
    reset_var()
    result = serialize_fn_generator(msg_context, object_array_spec, is_numpy)
    compare_file(array_d, 'object_varlen_ser_full.txt', result)
    reset_var()

