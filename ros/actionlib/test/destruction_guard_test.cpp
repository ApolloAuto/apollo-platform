/*********************************************************************
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/

//! \author Vijay Pradeep

#include <gtest/gtest.h>
#include <actionlib/destruction_guard.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace actionlib;

class TestRunner : public testing::Test
{
public:
  TestRunner() : done_protecting_(false)
  {

  }

  void protectingThread()
  {
    DestructionGuard::ScopedProtector protector_1(guard_);
    EXPECT_TRUE(protector_1.isProtected());

    DestructionGuard::ScopedProtector protector_2(guard_);
    EXPECT_TRUE(protector_2.isProtected());

    // Let the main thread know that we've successfully created to protectors
    {
      boost::mutex::scoped_lock lock(mutex_);
      done_protecting_ = true;
      cond_.notify_all();
    }

    // Don't destruct the protectors immeadiately. Sleep for a little bit, and then destruct.
    //  This will force the main thread to have to wait in it's destruct() call
    printf("protecting thread is sleeping\n");
    usleep(5000000);
    printf("protecting thread is exiting\n");
  }

protected:
  DestructionGuard guard_;

  bool done_protecting_;
  boost::mutex mutex_;
  boost::condition cond_;
};


TEST_F(TestRunner, threaded_test)
{
  boost::thread spin_thread(boost::bind(&TestRunner::protectingThread, this));

  {
    boost::mutex::scoped_lock lock(mutex_);
    while (!done_protecting_)
    {
      cond_.timed_wait(lock, boost::posix_time::milliseconds(100.0f));
    }
  }

  printf("About to destruct\n");
  guard_.destruct();
  printf("Done destructing\n");

  // Already 'destructed', so protector should fail
  DestructionGuard::ScopedProtector protector(guard_);
  EXPECT_FALSE(protector.isProtected());

  spin_thread.join();
}


TEST(DestructionGuard, easy_test)
{
  DestructionGuard guard;

  {
    DestructionGuard::ScopedProtector protector_1(guard);
    EXPECT_TRUE(protector_1.isProtected());

    DestructionGuard::ScopedProtector protector_2(guard);
    EXPECT_TRUE(protector_2.isProtected());
  }

  guard.destruct();

  {
    DestructionGuard::ScopedProtector protector_3(guard);
    EXPECT_FALSE(protector_3.isProtected());
  }
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);


  return RUN_ALL_TESTS();
}
