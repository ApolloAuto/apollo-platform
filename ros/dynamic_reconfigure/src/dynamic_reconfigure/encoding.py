# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
    import roslib; roslib.load_manifest('dynamic_reconfigure')
except:
    pass
import rospy
import inspect
import copy
import sys

from dynamic_reconfigure.msg import Config as ConfigMsg
from dynamic_reconfigure.msg import ConfigDescription as ConfigDescrMsg
from dynamic_reconfigure.msg import Group as GroupMsg
from dynamic_reconfigure.msg import GroupState
from dynamic_reconfigure.msg import IntParameter, BoolParameter, StrParameter, DoubleParameter, ParamDescription

class Config(dict):
    def __init__(self, *args, **kwargs):
        dict.__init__(self, *args, **kwargs)

    def __getstate__(self):
        return self.__dict__.items()

    def __setstate__(self, items):
        for key, val in items:
            self.__dict__[key] = val

    def __repr__(self):
        return super(Config, self).__repr__() 

    def __setitem__(self, key, value):
        return super(Config, self).__setitem__(key, value)

    def __getitem__(self, name):
        return super(Config, self).__getitem__(name)

    def __delitem__(self, name):
        return super(Config, self).__delitem__(name)

    __getattr__ = __getitem__
    __setattr__ = __setitem__

    def copy(self):
        return Config(self)

    def __deepcopy__(self, memo):
        c = type(self)({})
        for key, value in self.iteritems():
            c[copy.deepcopy(key)] = copy.deepcopy(value)

        return c
        

def encode_description(descr):
    msg = ConfigDescrMsg()
    msg.max = encode_config(descr.max)
    msg.min = encode_config(descr.min)
    msg.dflt = encode_config(descr.defaults)
    msg.groups = encode_groups(None, descr.config_description)
    return msg

def encode_groups(parent, group):
    group_list = []
    
    msg = GroupMsg()

    msg.name = group['name']
    msg.id = group['id']
    msg.parent = group['parent']
    msg.type = group['type']

    for param in group['parameters']:
        msg.parameters.append(ParamDescription(param['name'], param['type'], param['level'], param['description'], param['edit_method']))

    group_list.append(msg)
    for next in group['groups']:
        group_list.extend(encode_groups(msg, next))

    return group_list

def encode_config(config, flat=True):
    msg = ConfigMsg()
    for k, v in config.items():
        ## @todo add more checks here?
        if   type(v) == int:   msg.ints.append(IntParameter(k, v))
        elif type(v) == bool:  msg.bools.append(BoolParameter(k, v))
        elif type(v) == str:   msg.strs.append(StrParameter(k, v))
        elif sys.version_info.major < 3 and type(v) == unicode:
            msg.strs.append(StrParameter(k, v))
        elif type(v) == float: msg.doubles.append(DoubleParameter(k, v))
        elif type(v) == dict or isinstance(v, Config):
            if flat is True:
                def flatten(g):
                    groups = []
                    for name, group in g['groups'].items():
                        groups.extend(flatten(group))
                        groups.append(GroupState(group['name'], group['state'], group['id'], group['parent']))
                    return groups
                msg.groups.append(GroupState(v['name'], v['state'], v['id'], v['parent']))
                msg.groups.extend(flatten(v))
            else:
                msg.groups = [GroupState(x['name'], x['state'], x['id'], x['parent']) for x in v]

    return msg

def group_dict(group):
    try:
        state = group.state
    except AttributeError:
        state = True
    if hasattr(group, 'type'):
        type = group.type
    else:
        type =''
    return Config({
        'id' : group.id,
        'parent' : group.parent,
        'name' : group.name,
        'type' : type,
        'state': state,
        'groups' : Config({}),
        'parameters' : Config({}),
    })

def decode_description(msg):
    mins = decode_config(msg.min)
    maxes = decode_config(msg.max)
    defaults = decode_config(msg.dflt)
    groups = {}
    grouplist = msg.groups

    def params_from_msg(msg):
        params = []
        for param in msg.parameters:
            name = param.name
            params.append({
               'name': name,
               'min' : mins[name],
               'max' : maxes[name],
               'default' : defaults[name],
               'type' : param.type,
               'level': param.level,
               'description' : param.description,
               'edit_method' : param.edit_method,
            })
        return params

    # grab the default group
    for group in grouplist:
        if group.id == 0:
            groups = group_dict(group)
            groups['parameters'] = params_from_msg(group)
  
    def build_tree(group):
        children = Config({})
        for g in grouplist:
            if g.id == 0:
               pass
            elif g.parent == group['id']:
               gd = group_dict(g)
               
               gd['parameters'] = params_from_msg(g)
               gd['groups'] = build_tree(gd)
               # add the dictionary into the tree
               children[gd['name']] = gd
        return children

    groups['groups'] = build_tree(groups)

    return groups

def get_tree(m, group = None):
    if group is None:
        for x in m.groups:
            if x.id == 0:
                group = x

    children = Config({})
    for g in m.groups:
        if g.id == 0:
          pass
        elif g.parent == group.id:
            gd = group_dict(g)

            gd['groups'] = get_tree(m, g)
            children[gd['name']] = gd

    if group.id == 0:
        ret = group_dict(group)
        ret['groups'] = children
        return ret
    else:
        return children

def initial_config(msg, description = None):
    d = Config([(kv.name, kv.value) for kv in msg.bools + msg.ints + msg.strs + msg.doubles])
    def gt(m, descr, group = None):
        # get the default group
        if group is None:
            for x in m.groups:
                if x.id == 0:
                    group = x

        children = Config({})
        for g in m.groups:
            if g.id == 0:
                pass
            elif g.parent == group.id:
                gd = group_dict(g)

                def find_state(gr, dr):
                    for g in dr['groups']:
                        if g['id'] == gr['id']:
                            gr['state'] = g['state']
                            return
                        else:
                            find_state(gr, g)
                            return

                find_state(gd, descr)

                # Get the tree for this group
                gd['groups'] = gt(m, descr, g)
                children[gd['name']] = gd

        if group.id == 0:
            ret = group_dict(group)
            ret['groups'] = children
            return ret
        else:
            return children

    if not msg.groups == [] and description is not None:
        d["groups"] = gt(msg, description)

        def add_params(group, descr):
            for param in descr['parameters']:
                group['parameters'][param['name']] = d[param['name']]
            for n, g in group['groups'].items():
                for dr in descr['groups']:
                    if dr['name'] == g['name']:
                        add_params(g, dr)

        add_params(d['groups'], description)

    return d 

def decode_config(msg, description = None):
    if sys.version_info.major < 3:
        for s in msg.strs:
            if not isinstance(s.value, unicode):
                try:
                    s.value.decode('ascii')
                except UnicodeDecodeError:
                    s.value = s.value.decode('utf-8')

    d = Config([(kv.name, kv.value) for kv in msg.bools + msg.ints + msg.strs + msg.doubles])
    if not msg.groups == [] and description is not None:
        d["groups"] = get_tree(msg)
        
        def add_params(group, descr):
            for param in descr['parameters']:
                if param['name'] in d.keys():
                    group[param['name']] = d[param['name']]
            for n, g in group['groups'].items():
                for nr, dr in descr['groups'].items():
                    if dr['name'] == g['name']:
                        add_params(g, dr)

        add_params(d['groups'], description)

    return d 

def extract_params(group):
    params = []
    params.extend(group['parameters'])
    try:
        for n,g in group['groups'].items():
            params.extend(extract_params(g))
    except AttributeError:
        for g in group['groups']:
            params.extend(extract_params(g))
    return params

def get_parents(group, descriptions):
    parents = []
    for p in descriptions['group']:
        if p['id'] == group['parent']:
            parents.extend(get_parents(p, descriptions))
            parents.append(p)
    return parents
