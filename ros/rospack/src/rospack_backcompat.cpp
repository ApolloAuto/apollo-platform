/*
 * Copyright (C) 2008, Willow Garage, Inc., Morgan Quigley
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "rospack/rospack_backcompat.h"
#include "rospack/rospack.h"
#include "rospack_cmdline.h"

#include <boost/algorithm/string.hpp>
#include <string.h>
#include <stdio.h>
#include <errno.h>

namespace rospack
{

int
ROSPack::run(int argc, char** argv)
{
  // allow caching of results between calls (of same process)
  static rospack::Rospack rp;
  output_.clear();
  bool success = rospack::rospack_run(argc, argv, rp, output_);
  if(!success)
  {
    fprintf(stderr, "[librospack]: error while executing command\n");
    return 1;
  }
  return 0;
}

int
ROSPack::run(const std::string& cmd)
{
  // Callers of this method don't make 'rospack' argv[0].
  std::string full_cmd = std::string("rospack ") + cmd;

  int argc;
  char** argv;
  std::vector<std::string> full_cmd_split;
  boost::split(full_cmd_split, full_cmd,
               boost::is_any_of(" "),
               boost::token_compress_on);
  argc = full_cmd_split.size();
  argv = new char*[argc];
  int i = 0;
  for(std::vector<std::string>::const_iterator it = full_cmd_split.begin();
      it != full_cmd_split.end();
      ++it)
  {
    argv[i] = new char[it->size()+1];
    memset(argv[i], 0, it->size()+1);
    memcpy(argv[i], it->c_str(), it->size());
    i++;
  }

  int ret = run(argc, argv);

  for(int i=0; i<argc; i++)
    delete[] argv[i];
  delete[] argv;

  return ret;
}

} // namespace rospack
