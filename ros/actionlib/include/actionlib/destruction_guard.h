/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
#ifndef ACTIONLIB_DESTRUCTION_GUARD_
#define ACTIONLIB_DESTRUCTION_GUARD_

#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>

namespace actionlib {
  /**
   * @class DestructionGuard
   * @brief This class protects an object from being destructed until all users of that object relinquish control of it
   */
  class DestructionGuard {
    public:
      /**
       * @brief  Constructor for a DestructionGuard
       */
      DestructionGuard() : use_count_(0), destructing_(false){}
      void destruct(){
        boost::mutex::scoped_lock lock(mutex_);
        destructing_ = true;
        while(use_count_ > 0){
          count_condition_.timed_wait(lock, boost::posix_time::milliseconds(1000.0f)); 
        }
      }

      /**
       * @brief  Attempts to protect the guarded object from being destructed
       * @return  True if protection succeeded, false if protection failed
       */
      bool tryProtect(){
        boost::mutex::scoped_lock lock(mutex_);
        if(destructing_)
          return false;
        use_count_++;
        return true;
      }

      /**
       * @brief  Releases protection on the guarded object
       */
      void unprotect(){
        boost::mutex::scoped_lock lock(mutex_);
        use_count_--;
      }

      /**
       * @class ScopedProtector
       * @brief Protects a DestructionGuard until this object goes out of scope
       */
      class ScopedProtector
      {
        public:
          /**
           * @brief  Constructor for a ScopedProtector
           * @param guard The DestructionGuard to protect
           */
          ScopedProtector(DestructionGuard& guard) : guard_(guard), protected_(false){
            protected_ = guard_.tryProtect();
          }

          /**
           * @brief  Checks if the ScopedProtector successfully protected the DestructionGuard
           * @return True if protection succeeded, false otherwise
           */
          bool isProtected(){
            return protected_;
          }

          /**
           * @brief  Releases protection of the DestructionGuard if necessary
           */
          ~ScopedProtector(){
            if(protected_)
              guard_.unprotect();
          }

        private:
          DestructionGuard& guard_;
          bool protected_;
      };


    private:
      boost::mutex mutex_;
      int use_count_;
      bool destructing_;
      boost::condition count_condition_;
  };

};
#endif
