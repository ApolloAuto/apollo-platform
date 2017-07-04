/*
 * Copyright (C) 2008, Willow Garage, Inc.
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

#include <string>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/tr1/unordered_set.hpp>

#include "utils.h"

namespace rospack
{

void
deduplicate_tokens(const std::string& instring,
                   bool last,
                   std::string& outstring)
{
  std::vector<std::string> vec;
  std::tr1::unordered_set<std::string> set;
  boost::split(vec, instring,
               boost::is_any_of("\t "),
               boost::token_compress_on);
  if(last)
    std::reverse(vec.begin(), vec.end());
  std::vector<std::string> vec_out;
  for(std::vector<std::string>::const_iterator it = vec.begin();
      it != vec.end();
      ++it)
  {
    if(set.find(*it) == set.end())
    {
      vec_out.push_back(*it);
      set.insert(*it);
    }
  }
  if(last)
    std::reverse(vec_out.begin(), vec_out.end());
  for(std::vector<std::string>::const_iterator it = vec_out.begin();
      it != vec_out.end();
      ++it)
  {
    if(it == vec_out.begin())
      outstring.append(*it);
    else
      outstring.append(std::string(" ") + *it);
  }
}

void
parse_compiler_flags(const std::string& instring,
                     const std::string& token,
                     bool select,
                     bool last,
                     std::string& outstring)
{
  std::string intermediate;
  std::vector<std::string> result_vec;
  boost::split(result_vec, instring,
               boost::is_any_of("\t "),
               boost::token_compress_on);
  for(std::vector<std::string>::const_iterator it = result_vec.begin();
      it != result_vec.end();
      ++it)
  {
    // Combined into one arg
    if(it->size() > token.size() && it->substr(0,token.size()) == token)
    {
      if(select)
      {
        if(intermediate.size())
          intermediate.append(" ");
        intermediate.append(it->substr(token.size()));
      }
    }
    // Space-separated
    else if((*it) == token)
    {
      std::vector<std::string>::const_iterator iit = it;
      if(++iit != result_vec.end())
      {
        if(it->size() >= token.size() && it->substr(0,token.size()) == token)
        {
          // skip it
        }
        else
        {
          if(select)
          {
            if(intermediate.size())
              intermediate.append(" ");
            intermediate.append((*iit));
          }
          it = iit;
        }
      }
    }
    // Special case: if we're told to look for -l, then also find *.a
    else if(it->size() > 2 &&
            (*it)[0] == '/' &&
            it->substr(it->size()-2) == ".a")
    {
      if(select)
      {
        if(intermediate.size())
          intermediate.append(" ");
        intermediate.append((*it));
      }
    }
    else if(!select)
    {
      if(intermediate.size())
        intermediate.append(" ");
      intermediate.append((*it));
    }
  }
  if(select)
    deduplicate_tokens(intermediate, last, outstring);
  else
    outstring = intermediate;
}

}

