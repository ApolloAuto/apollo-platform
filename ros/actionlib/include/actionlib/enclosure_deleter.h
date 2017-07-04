/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/

#include <boost/shared_ptr.hpp>

#ifndef ACTIONLIB_ENCLOSURE_DELETER_H_
#define ACTIONLIB_ENCLOSURE_DELETER_H_

namespace actionlib
{

/*
 * This allows the creation of a shared pointer to a section
 * of an already reference counted structure. For example,
 * if in the following picture Enclosure is reference counted with
 * a boost::shared_ptr and you want to return a boost::shared_ptr
 * to the Member that is referenced counted along with Enclosure objects
 *
 * Enclosure ---------------  <--- Already reference counted
 * -----Member <------- A member of enclosure objects, eg. Enclosure.Member
 */
template <class Enclosure> class EnclosureDeleter {
  public:
    EnclosureDeleter(const boost::shared_ptr<Enclosure>& enc_ptr) : enc_ptr_(enc_ptr){}

    template<class Member> void operator()(Member* member_ptr){
      enc_ptr_.reset();
    }

  private:
    boost::shared_ptr<Enclosure> enc_ptr_;
};

template <class Enclosure, class Member>
boost::shared_ptr<Member> share_member(boost::shared_ptr<Enclosure> enclosure, Member &member)
{
  EnclosureDeleter<Enclosure> d(enclosure);
  boost::shared_ptr<Member> p(&member, d);
  return p;
}

}

#endif
