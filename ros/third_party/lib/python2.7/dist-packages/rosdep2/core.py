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

from __future__ import print_function

import os
import sys
import traceback

def rd_debug(s):
    if "ROSDEP_DEBUG" in os.environ:
        print(s)

def print_bold(msg):
    """
    print message printed to screen with bold decoration for greater clarity
    :param msg: message to print, ``str``
    """
    if sys.platform in ['win32']:
        print('%s'%msg)  #windows console is terrifically boring 
    else:
        print('\033[1m%s\033[0m'%msg)
    
class InvalidData(Exception):
    """
    Data is not in valid rosdep format.
    """

    def __init__(self, message, origin=None):
        super(InvalidData, self).__init__(message)
        self.origin = origin

class UnsupportedOs(Exception):
    pass
    
class RosdepInternalError(Exception):

    def __init__(self, e, message=None):
        self.error = e
        if message is None:
            self.message = traceback.format_exc()
        else:
            self.message = message

    def __str__(self):
        return self.message

class CachePermissionError(Exception):

    """Failure when writing the cache."""

    pass
        
class DownloadFailure(Exception):
    """
    Failure downloading sources list data for I/O or other format reasons.
    """
    pass

class InstallFailed(Exception):

    def __init__(self, failure=None, failures=None):
        """
        One of failure/failures must be set.
        
        :param failure: single (installer_key, message) tuple.  
        :param failures: list of (installer_key, message) tuples
        """
        if failures is not None:
            self.failures = failures
        elif not failure:
            raise ValueError("failure is None")
        else:
            self.failures = [failure]
    
    def __str__(self):
        return '\n'.join(['%s: %s'%(key, message) for (key, message) in self.failures])
