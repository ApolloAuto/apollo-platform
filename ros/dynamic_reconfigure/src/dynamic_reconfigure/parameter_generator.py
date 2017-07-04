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

# Author: Blaise Gassend

# Given a set of parameters, generates the messages, service types, and
# classes to allow runtime reconfiguration. Documentation of a node's
# parameters is a handy byproduct.

## @todo
# Need to check types of min max and default
# Need to put sane error on exceptions

import roslib; roslib.load_manifest("dynamic_reconfigure")
import roslib.packages
from string import Template
import os
import inspect
import string 
import sys
import re

#LINEDEBUG="#line"
LINEDEBUG="//#line"

# Convenience names for types
str_t = "str"
bool_t = "bool"
int_t = "int"
double_t = "double"

id = 0


def check_description(description):
    quotes = ['"', "'"]
    for quote in quotes:
        if description.find(quote) != -1:
            raise Exception(r"""quotes not allowed in description string `%s`""" % description)


def check_name(name):
    pattern = r'^[a-zA-Z][a-zA-Z0-9_]*$'
    if not re.match(pattern, name):
        raise Exception("The name of field \'%s\' does not follow the ROS naming conventions, see http://wiki.ros.org/ROS/Patterns/Conventions"%name)


class ParameterGenerator:
    minval = {
            'int' : -0x80000000, #'INT_MIN',
            'double' : -1e10000,#'-std::numeric_limits<double>::infinity()',
            'str' : '',
            'bool' : False,
            }
            
    maxval = {
            'int' : 0x7FFFFFFF, #'INT_MAX',
            'double' : 1e10000, #'std::numeric_limits<double>::infinity()',
            'str' : '',
            'bool' : True,
            }
    
    defval = {
            'int' : 0,
            'double' : 0,
            'str' : '',
            'bool' : False,
            }
        
    class Group:
        instances = {}
        def __init__(self, gen, name, type, state, id, parent):
            self.name = name.replace(" ", "_")
            self.type = type
            self.groups = []
            self.parameters =[]
            self.gen = gen
            self.id = id
            self.parent = parent
            self.state = state

            self.srcline = inspect.currentframe().f_back.f_lineno
            self.srcfile = inspect.getsourcefile(inspect.currentframe().f_back.f_code)

            self.instances[self.id] = self

        def get_group(self, id):
            return self.instances[id]

        def add_group(self, name, type="", state=True):
            global id
            group = self.gen.Group(self.gen, name, type, state, id, self.id)
            id = id + 1
            self.groups.append(group)
            return group

        def add(self, name, paramtype, level, description, default = None, min = None, max = None, edit_method = ""):
            newparam = {
                    'name' : name,
                    'type' : paramtype,
                    'default' : default,
                    'level' : level,
                    'description' : description,
                    'min' : min,
                    'max' : max,
                    'srcline' : inspect.currentframe().f_back.f_lineno,
                    'srcfile' : inspect.getsourcefile(inspect.currentframe().f_back.f_code),
                    'edit_method' : edit_method,
            }
            if type == str_t and (max != None or min != None):
                raise Exception("Max or min specified for %s, which is of string type"%name)

            check_name(name)
            check_description(description)
            self.gen.fill_type(newparam)
            self.gen.check_type_fill_default(newparam, 'default', self.gen.defval[paramtype])
            self.gen.check_type_fill_default(newparam, 'max', self.gen.maxval[paramtype])
            self.gen.check_type_fill_default(newparam, 'min', self.gen.minval[paramtype])

            self.parameters.append(newparam)

        # Compile a list of all the parameters in this group
        def get_parameters(self):
            params = []
            params.extend(self.parameters)
            for group in self.groups:
                params.extend(group.get_parameters())

            return params

        def get_parents(self):
            parents = []
            if not self.id == 0:
                p = self.get_group(self.parent)
                parents.extend(p.get_parents())
                parents.append(self.name)
            else:
                parents.append(self.name)
            return parents

        def get_field(self):
            fld = []
            fld.extend(self.get_parents())
            ret = []
            for x in fld:
                if x == self.name:
                    ret.append(string.lower(x))
                else:
                    ret.append(string.upper(x))
            return string.join(ret, "::")
            
        def get_class(self, parent = False):
            cls = []
            cls.extend(self.get_parents())
            cls = [string.upper(x) for x in cls]
            if parent == True:
                cls.pop()
            return string.join(cls, "::")

        # dictionary used to create the generated classes
        def to_dict(self):
          if self.id == 0:
              name = "groups"
          else:
              name = self.name
          if self.state:
              state = 'true'
          else:
              state = 'false'
          return {
              'name': self.name,
              'type': self.type,
              'state': self.state,
              'cstate': state,
              'id':self.id, 'parent':self.parent,
              'parameters': self.parameters,
              'groups' : [group.to_dict() for group in self.groups],
              'srcline' : self.srcline,
              'srcfile' : self.srcfile,
              'class' : self.get_class(),
              'parentclass': self.get_class(parent=True),
              'parentname': self.get_group(self.parent).name,
              'field' : self.get_field(),
              'upper': string.upper(self.name),
              'lower': string.lower(name)
          }


    def pytype(self, drtype):
      return { 'str':str, 'int':int, 'double':float, 'bool':bool }[drtype]

    def check_type(self, param, field):
        drtype = param['type']
        name = param['name']
        value = param[field]
        pytype = self.pytype(drtype)
        if pytype != type(value) and (pytype != float or type(value) != int):
            raise TypeError("'%s' has type %s, but %s is %s"%(name, drtype, field, repr(value)))
        param[field] = pytype(value)

    def fill_type(self, param):
        param['ctype'] = { 'str':'std::string', 'int':'int', 'double':'double', 'bool':'bool' }[param['type']]
        param['cconsttype'] = { 'str':'const char * const', 'int':'const int', 'double':'const double', 'bool':'const bool' }[param['type']]

    def check_type_fill_default(self, param, field, default):
        value = param[field]
        # If no value, use default.
        if value == None:
            param[field] = default
            return
        # Check that value type is compatible with type.
        self.check_type(param, field)
    
    def __init__(self):
        global id
        self.group = self.Group(self, "Default", "", True, 0, 0)
        id = 1
        self.constants = []
        self.dynconfpath = roslib.packages.get_pkg_dir("dynamic_reconfigure")

    def const(self, name, type, value, descr):
        newconst = { 
                'name':name, 
                'type':type, 
                'value':value,
                'srcline' : inspect.currentframe().f_back.f_lineno,
                'srcfile' : inspect.getsourcefile(inspect.currentframe().f_back.f_code),
                'description' : descr
                }
        check_description(descr)
        self.fill_type(newconst)
        self.check_type(newconst, 'value')
        self.constants.append(newconst)
        return newconst # So that we can assign the value easily.

    def enum(self, constants, description):
        if len(set(const['type'] for const in constants)) != 1:
            raise Exception("Inconsistent types in enum!")
        check_description(description)
        return repr({ 'enum' : constants, 'enum_description' : description }) 

    # Wrap add and add_group for the default group
    def add(self, name, paramtype, level, description, default = None, min = None, max = None, edit_method = ""):
        self.group.add(name, paramtype, level, description, default, min, max, edit_method) 

    def add_group(self, name, type="", state=True):
        return self.group.add_group(name, type=type, state=state)

    def mkdirabs(self, path):
        if os.path.isdir(path):
            pass
        elif os.path.isfile(path):
            raise OSError("Error creating directory %s, a file with the same name exists" %path)
        else:
            head, tail = os.path.split(path)
            if head and not os.path.isdir(head):
                self.mkdir(head)
            if tail:
                try:
                    os.mkdir(path)
                except OSError:
                    if not os.path.isdir(path):
                        raise

    def mkdir(self, path):
        path = os.path.join(self.pkgpath, path)
        self.mkdirabs(path)

    def generate(self, pkgname, nodename, name):
        self.pkgname = pkgname
        self.pkgpath = roslib.packages.get_pkg_dir(pkgname)
        self.name = name
        self.nodename = nodename
        self.msgname = name+"Config"

        # Don't regenerate headers if the config hasn't been modfied
        cpp_header = os.path.realpath(os.path.join(self.pkgpath, "cpp", pkgname, self.msgname + ".h"))      
        if os.path.exists(cpp_header) and os.path.getmtime(os.path.realpath(__file__)) < os.path.getmtime(cpp_header):
              exit(0)

        try:
            if sys.modules['__main__']._DYNAMIC_RECONFIGURE_GENERATING_DEPENDENCIES:
                # Done this way because importing this module from gendeps
                # causes imports of dynamic_reconfigure.msg to fail from at
                # least some .cfg files. (Not sure why)
                return
        except:
            pass
        try:
            #print '**************************************************************'
            #print '**************************************************************'
            print Template("Generating reconfiguration files for $name in $pkgname").\
                    substitute(name=self.name, pkgname = self.pkgname)
            #print '**************************************************************'
            #print '**************************************************************'
            self.generatecpp()
            self.generatedoc()
            self.generatewikidoc()
            self.generateusage()
            self.generatepy()
            self.deleteobsolete()
        except Exception, e:
            print "Error building srv %s.srv"%name
            import traceback
            traceback.print_exc()
            exit(1)

    def generatewikidoc(self):
        self.mkdir("docs")
        f = open(os.path.join(self.pkgpath, "docs", self.msgname+".wikidoc"), 'w')
        print >> f, \
"""# Autogenerated param section. Do not hand edit.
param {
group.0 {
name=Dynamically Reconfigurable Parameters
desc=See the [[dynamic_reconfigure]] package for details on dynamically reconfigurable parameters."""
        i=-1
        for param in self.group.get_parameters():
            i=i+1
            range = ""
            try:
              enum = eval(param['edit_method'])['enum']
              range = ", ".join(Template("$name ($value): $description").substitute(const) for const in enum)
              range = "Possible values are: " + range
            except:
              if param['type'] == int_t or param['type'] == double_t:
                  range = Template("Range: $min to $max").substitute(param)
            print >> f, Template(
"""$i.name= ~$name
$i.default= $default
$i.type= $type
$i.desc=$description $range"""
).substitute(param, range = range, i = i)
        print >> f,"}\n}\n# End of autogenerated section. You may edit below."
        f.close()

    def generateusage(self):
        self.mkdir("docs")
        f = open(os.path.join(self.pkgpath, "docs", self.msgname+"-usage.dox"), 'w')
        #print >> f, "/**"
        print >> f, "\\subsubsection usage Usage"
        print >> f, '\\verbatim'
        print >> f, Template('<node name="$nodename" pkg="$pkgname" type="$nodename">').\
                substitute(pkgname = self.pkgname, nodename = self.nodename)
        for param in self.group.get_parameters():
            print >> f, Template('  <param name="$name" type="$type" value="$default" />').substitute(param)
        print >> f, '</node>'
        print >> f, '\\endverbatim'
        print >> f
        #print >> f, "*/"
        f.close()
    
    def generatedoc(self):
        self.mkdir("docs")
        f = open(os.path.join(self.pkgpath, "docs", self.msgname+".dox"), 'w')
        #print >> f, "/**"
        print >> f, "\\subsubsection parameters ROS parameters"
        print >> f
        print >> f, "Reads and maintains the following parameters on the ROS server"
        print >> f
        for param in self.group.get_parameters():
            print >> f, Template("- \\b \"~$name\" : \\b [$type] $description min: $min, default: $default, max: $max").substitute(param)
        print >> f
        #print >> f, "*/"
        f.close()

    def generateusage(self):
        self.mkdir("docs")
        f = open(os.path.join(self.pkgpath, "docs", self.msgname+"-usage.dox"), 'w')
        #print >> f, "/**"
        print >> f, "\\subsubsection usage Usage"
        print >> f, '\\verbatim'
        print >> f, Template('<node name="$nodename" pkg="$pkgname" type="$nodename">').\
                substitute(pkgname = self.pkgname, nodename = self.nodename)
        for param in self.group.get_parameters():
            print >> f, Template('  <param name="$name" type="$type" value="$default" />').substitute(param)
        print >> f, '</node>'
        print >> f, '\\endverbatim'
        print >> f
        #print >> f, "*/"
        f.close()

    def crepr(self, param, val):
        type = param["type"]
        if type == 'str':
            return '"'+val+'"'
        if type == 'int':
            return str(val)
        if type == 'double':
            if val == float('inf'):
                return 'std::numeric_limits<double>::infinity()'
            elif val == -float('inf'):
                return '-std::numeric_limits<double>::infinity()'
            else:
                return str(val)
        if  type == 'bool':
            return { True : 1, False : 0 }[val]
        raise TypeError(type)
#        if type == 'string':
#            return '"'+val+'"'
#        if 'uint' in type:
#            return str(val)+'ULL'
#        if 'int' in type:
#            return str(val)+'LL'
#        if 'time' in type:
#            return 'ros::Time('+str(val)+')'
#        if 'duration' in type:
#            return 'ros::Duration('+str(val)+')'
#        if  'float' in types:
#            return str(val)

    def appendline(self, list, text, param, value = None):
        if value == None:
            val = ""
        else:
            val = self.crepr(param, param[value])
        list.append(Template('${doline} $srcline "$srcfile"\n      '+text).safe_substitute(param, v=val, doline=LINEDEBUG, configname=self.name))
    
    def appendgroup(self, list, group):
        subgroups = []
        for g in group.groups:
            self.appendgroup(subgroups, g)
        setters = []
        params = []
        for p in group.parameters:
            setters.append(Template("        if(\"${name}\"==(*_i)->name){${name} = boost::any_cast<${ctype}>(val);}").substitute(p));
            params.append(Template("${ctype} ${name};").substitute(p));

        subgroups = string.join(subgroups, "\n") 
        setters = string.join(setters, "\n")
        params = string.join(params, "\n")
        grouptemplate = open(os.path.join(self.dynconfpath, "templates", "GroupClass.h.template")).read()
        list.append(Template(grouptemplate).safe_substitute(group.to_dict(), subgroups = subgroups, setters = setters, params = params, configname = self.name))

    def generatecpp(self):
        # Read the configuration manipulator template and insert line numbers and file name into template.
        templatefile = os.path.join(self.dynconfpath, "templates", "ConfigType.h.template")
        templatelines = []
        templatefilesafe = templatefile.replace('\\', '\\\\') # line directive does backslash expansion.
        curline = 1
        f = open(templatefile)
        for line in f:
            curline = curline + 1
            templatelines.append(Template(line).safe_substitute(linenum=curline,filename=templatefilesafe))
        f.close()
        template = ''.join(templatelines)
        
        # Write the configuration manipulator.
        cfg_cpp_dir = os.path.join("cfg", "cpp", self.pkgname)
        self.mkdir(cfg_cpp_dir)
        f = open(os.path.join(self.pkgpath, cfg_cpp_dir, self.name+"Config.h"), 'w')

        paramdescr = []
        groups = []
        members = []
        constants = []

        for const in self.constants:
            self.appendline(constants, "${cconsttype} ${configname}_${name} = $v;", const, "value")

        def write_params(group):
            if group.id == 0:
                paramdescr.append(Template("${configname}Config::GroupDescription<${configname}Config::${class}, ${configname}Config> ${name}(\"${name}\", \"${type}\", ${parent}, ${id}, ${cstate}, &${configname}Config::${lower});").safe_substitute(group.to_dict(), configname = self.name))
            else:
                paramdescr.append(Template("${configname}Config::GroupDescription<${configname}Config::${class}, ${configname}Config::${parentclass}> ${name}(\"${name}\", \"${type}\", ${parent}, ${id}, ${cstate}, &${configname}Config::${field});").safe_substitute(group.to_dict(), configname = self.name))
            for param in group.parameters:
                self.appendline(members, "${ctype} ${name};", param)
                self.appendline(paramdescr, "__min__.${name} = $v;", param, "min")
                self.appendline(paramdescr, "__max__.${name} = $v;", param, "max")
                self.appendline(paramdescr, "__default__.${name} = $v;", param, "default")
                self.appendline(paramdescr, group.to_dict()['name']+".abstract_parameters.push_back(${configname}Config::AbstractParamDescriptionConstPtr(new ${configname}Config::ParamDescription<${ctype}>(\"${name}\", \"${type}\", ${level}, "\
                        "\"${description}\", \"${edit_method}\", &${configname}Config::${name})));", param)
                self.appendline(paramdescr, 
                        "__param_descriptions__.push_back(${configname}Config::AbstractParamDescriptionConstPtr(new ${configname}Config::ParamDescription<${ctype}>(\"${name}\", \"${type}\", ${level}, "\
                        "\"${description}\", \"${edit_method}\", &${configname}Config::${name})));", param)
                
            for g in group.groups:
                write_params(g)    
            
            self.appendline(paramdescr, "${name}.convertParams();", group.to_dict())
            if group.id == 0:
                self.appendline(paramdescr, "__group_descriptions__.push_back(${configname}Config::AbstractGroupDescriptionConstPtr(new ${configname}Config::GroupDescription<${configname}Config::${class}, ${configname}Config>(${name})));", group.to_dict())
            else:
                self.appendline(paramdescr, "${parentname}.groups.push_back(${configname}Config::AbstractGroupDescriptionConstPtr(new ${configname}Config::GroupDescription<${configname}Config::${class}, ${configname}Config::${parentclass}>(${name})));", group.to_dict())
                self.appendline(paramdescr, "__group_descriptions__.push_back(${configname}Config::AbstractGroupDescriptionConstPtr(new ${configname}Config::GroupDescription<${configname}Config::${class}, ${configname}Config::${parentclass}>(${name})));", group.to_dict())

        write_params(self.group)
        self.appendgroup(groups, self.group)

        paramdescr = string.join(paramdescr, '\n')
        members = string.join(members, '\n')
        groups = string.join(groups, '\n')
        constants = string.join(constants, '\n')
        f.write(Template(template).substitute(uname=self.name.upper(), 
            configname=self.name, pkgname = self.pkgname, paramdescr = paramdescr,
            members = members, groups = groups, doline = LINEDEBUG, constants = constants))
        f.close()

    def deleteoneobsolete(self, file):
         try:
             os.unlink(file)
         except OSError:
             pass

    def deleteobsolete(self): ### @todo remove this after the transition period.
         self.deleteoneobsolete(os.path.join(self.pkgpath, "msg", self.msgname+".msg"))
         self.deleteoneobsolete(os.path.join("msg", "cpp", self.pkgpath, "msg", self.msgname+".msg"))
         self.deleteoneobsolete(os.path.join(self.pkgpath, "srv", "Get"+self.msgname+".srv"))
         self.deleteoneobsolete(os.path.join("srv", "cpp", self.pkgpath, "srv", "Get"+self.msgname+".srv"))
         self.deleteoneobsolete(os.path.join(self.pkgpath, "srv", "Set"+self.msgname+".srv"))
         self.deleteoneobsolete(os.path.join("srv", "cpp", self.pkgpath, "srv", "Set"+self.msgname+".srv"))

#    def msgtype(self, type):
#        return { 'int' : 'int32', 'bool' : 'int8', 'str' : 'string', 'double' : 'float64' }[type]
#
#    def generatemsg(self):
#        self.mkdir("msg")
#        f = open(os.path.join(self.pkgpath, "msg", self.msgname+".msg"), 'w')
#        print >> f, "# This is an autogerenated file. Please do not edit."
#        print >> f, ""
#        for param in self.parameters:
#            print >> f, Template("$type $name # $description").substitute(param, type=self.msgtype(param['type']))
#        f.close()
#
#    def generategetsrv(self):
#        self.mkdir("srv")
#        f = open(os.path.join(self.pkgpath, "srv", "Get"+self.msgname+".srv"), 'w')
#        print >> f, "# This is an autogerenated file. Please do not edit."
#        print >> f, ""
#        print >> f, "---" 
#        print >> f, self.msgname, "config", "# Current configuration of node."
#        print >> f, self.msgname, "defaults", "# Minimum values where appropriate."
#        print >> f, self.msgname, "min", "# Minimum values where appropriate."
#        print >> f, self.msgname, "max", "# Maximum values where appropriate."
#        f.close()
#
#    def generatesetsrv(self):
#        self.mkdir("srv")
#        f = open(os.path.join(self.pkgpath, "srv", "Set"+self.msgname+".srv"), 'w')
#        print >> f, "# This is an autogerenated file. Please do not edit."
#        print >> f, self.msgname, "config", "# Requested node configuration."
#        print >> f, "---"        
#        print >> f, self.msgname, "config", "# What the node's configuration was actually set to."
#        f.close()
    
    def generatepy(self):
        # Read the configuration manipulator template and insert line numbers and file name into template.
        templatefile = os.path.join(self.dynconfpath, "templates", "ConfigType.py.template")
        templatelines = []
        f = open(templatefile)
        template = f.read()
        f.close()
        
        # Write the configuration manipulator.
        self.mkdir(os.path.join("src", self.pkgname, "cfg"))
        f = open(os.path.join(self.pkgpath, "src", self.pkgname, "cfg", self.name+"Config.py"), 'w')
        f.write(Template(template).substitute(name = self.name, 
            pkgname = self.pkgname, pycfgdata = self.group.to_dict()))
        for const in self.constants:
            f.write(Template("${configname}_${name} = $v\n").
                    substitute(const, v = repr(const['value']), 
                        configname=self.name))
        f.close()

        f = open(os.path.join(self.pkgpath, "src", self.pkgname, "cfg", "__init__.py"), 'a')
        f.close()

        f = open(os.path.join(self.pkgpath, "src", self.pkgname, "__init__.py"), 'a')
        f.close()

        f = open(os.path.join(self.pkgpath, "src", self.pkgname, "__init__.py"), 'a')
        f.close()
