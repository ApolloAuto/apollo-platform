#!/usr/bin/env python
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

# Author: Alexander Sorokin. 

PKG='actionlib'
import rospy

import sys
import unittest
import threading

from actionlib import goal_id_generator,status_tracker
import actionlib_msgs.msg 

## A sample python unit test
class TestGoalIDGenerator(unittest.TestCase):

    def test_generator(self):
        gen1 = goal_id_generator.GoalIDGenerator();
        
        id1 = gen1.generate_ID()
        id2 = gen1.generate_ID()
        id3 = gen1.generate_ID()
        nn1,s1,ts1 = id1.id.split('-');
        nn2,s2,ts2 = id2.id.split('-');
        nn3,s3,ts3 = id3.id.split('-');

        self.assertEquals(nn1, "/test_goal_id_generator","node names are different")
        self.assertEquals(nn1, nn2, "node names are different")
        self.assertEquals(nn1, nn3, "node names are different")

        self.assertEquals(s1, "1", "Sequence numbers mismatch")
        self.assertEquals(s2, "2", "Sequence numbers mismatch")
        self.assertEquals(s3, "3", "Sequence numbers mismatch")


    def test_threaded_generation(self):
        """
        This test checks that all the ids are generated unique. This test should fail if the synchronization is set incorrectly. 
        @note this test is can succeed when the errors are present.
        """
        global ids_lists
        ids_lists={};
        def gen_ids(tID=1,num_ids=1000):
            gen = goal_id_generator.GoalIDGenerator();
            for i in range(0,num_ids):
                id=gen.generate_ID();
                ids_lists[tID].append(id);

        num_ids=1000;
        num_threads=200;
        threads=[];
        for tID in range(0,num_threads):
            ids_lists[tID]=[];
            t=threading.Thread(None,gen_ids,None,(),{'tID':tID,'num_ids':num_ids});
            threads.append(t);
            t.start();
        
        for t in threads:
            t.join();

        all_ids={};
        for tID in range(0,num_threads):
            self.assertEquals(len(ids_lists[tID]),num_ids)
            for id in ids_lists[tID]:
                nn,s,ts = id.id.split('-');
                self.assertFalse(s in all_ids,"Duplicate ID found");
                all_ids[s]=1;

    def test_status_tracker(self):
        tracker=status_tracker.StatusTracker("test-id",actionlib_msgs.msg.GoalStatus.PENDING);
        self.assertEquals(tracker.status.goal_id,"test-id");
        self.assertEquals(tracker.status.status,actionlib_msgs.msg.GoalStatus.PENDING);

        

if __name__ == '__main__':
    import rostest
    rospy.init_node("test_goal_id_generator")
    rostest.rosrun(PKG, 'test_goal_id_generator', TestGoalIDGenerator)

