# Copyright (c) 2012, Willow Garage, Inc.
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

try:
    from urllib.request import urlopen
except ImportError:
    from urllib2 import urlopen
import yaml
import warnings

from .core import DownloadFailure
from .rosdistrohelper import PreRep137Warning

# location of targets file for processing gbpdistro files
REP3_TARGETS_URL = 'https://raw.github.com/ros/rosdistro/master/releases/targets.yaml'

#seconds to wait before aborting download of gbpdistro data
DOWNLOAD_TIMEOUT = 15.0

def download_targets_data(targets_url=None):
    """
    Download REP 3 targets file and unmarshal from YAML.
    DEPRECATED: this function is deprecated. List of targets should be obtained
                from the rosdistro module.
                The body of this function is an example.

    :param target_url: override URL of platform targets file. Defaults
      to ``REP3_TARGETS_URL``.
    :raises: :exc:`DownloadFailure`
    :raises: :exc:`InvalidData` If targets file does not pass cursory validation checks.
    """
    warnings.warn("deprecated, use rosdistro instead", PreRep137Warning)

    if targets_url is None:
        targets_url = REP3_TARGETS_URL
    try:
        f = urlopen(targets_url, timeout=DOWNLOAD_TIMEOUT)
        text = f.read()
        f.close()
        targets_data = yaml.safe_load(text)
    except Exception as e:
        raise DownloadFailure("Failed to download target platform data for gbpdistro:\n\t%s"%(str(e)))
    if type(targets_data) == list:
        # convert to dictionary
        new_targets_data = {}
        for t in targets_data:
            platform = list(t.keys())[0]
            new_targets_data[platform] = t[platform]
        targets_data = new_targets_data
    return targets_data
