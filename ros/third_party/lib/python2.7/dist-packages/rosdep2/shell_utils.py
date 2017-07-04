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

# Author Tully Foote/tfoote@willowgarage.com

from __future__ import print_function

import os
import sys
import stat
import subprocess
import tempfile

from .core import rd_debug

if sys.hexversion > 0x03000000: #Python3
    python3 = True
else:
    python3 = False

def read_stdout(cmd):
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    std_out, std_err = p.communicate()
    if python3:
        return std_out.decode()
    else:
        return std_out    

def create_tempfile_from_string_and_execute(string_script, path=None, exec_fn=None):
    """
    :param path: (optional) path to temp directory, or ``None`` to use default temp directory, ``str``
    :param exec_fn: override subprocess.call with alternate executor (for testing)
    """
    if path is None:
        path = tempfile.gettempdir()
        
    result = 1
    try:
        fh = tempfile.NamedTemporaryFile('w', delete=False)
        fh.write(string_script)
        fh.close()
        print("Executing script below with cwd=%s\n{{{\n%s\n}}}\n"%(path, string_script))
        try:
            os.chmod(fh.name, stat.S_IRWXU)
            if exec_fn is None:
                result = subprocess.call(fh.name, cwd=path)
            else:
                result = exec_fn(fh.name, cwd=path)                
        except OSError as ex:
            print("Execution failed with OSError: %s"%(ex))
    finally:
        if os.path.exists(fh.name):
            os.remove(fh.name)
    
    rd_debug("Return code was: %s"%(result))
    return result == 0

