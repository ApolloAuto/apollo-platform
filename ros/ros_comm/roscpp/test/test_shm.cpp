/**
 * Unit test for SHM.
 *
*/
#include <gtest/gtest.h>
#include <stdlib.h>
#include <iostream>
#include <map>

#include "sharedmem_transport/sharedmem_block.h"
#include "sharedmem_transport/sharedmem_publisher_impl.h"
#include "sharedmem_transport/sharedmem_segment.h"
#include "sharedmem_transport/sharedmem_util.h"
#include "ros/time.h"
#include "boost/shared_ptr.hpp"

namespace ros {

TEST(SHMTest, handle_segment) {
    boost::shared_ptr<sharedmem_transport::SharedMemoryUtil> 
        util(new sharedmem_transport::SharedMemoryUtil());

    std::string topic = "/chatter";

    util->remove_segment(topic.c_str());

    boost::interprocess::managed_shared_memory* segment = util->get_segment(topic.c_str());
    EXPECT_EQ(NULL, segment);

    sharedmem_transport::SharedMemorySegment* segment_mgr = util->get_segment_mgr(segment);
    EXPECT_EQ(NULL, segment_mgr);

    bool ret = util->remove_segment(topic.c_str());
    ASSERT_FALSE(ret);

    boost::interprocess::managed_shared_memory* segment_sec = 
        util->create_segment(topic.c_str(), 1000000);
    ASSERT_TRUE(segment_sec != NULL);

    sharedmem_transport::SharedMemorySegment* segment_mgr_sec = 
        util->create_segment_mgr(segment_sec);
    ASSERT_TRUE(segment_mgr_sec != NULL);

    boost::interprocess::managed_shared_memory* segment_thd = util->get_segment(topic.c_str());
    ASSERT_TRUE(segment_thd != NULL);

    sharedmem_transport::SharedMemorySegment* segment_mgr_thd = util->get_segment_mgr(segment_thd);
    ASSERT_TRUE(segment_mgr_thd != NULL);

    bool ret_sec = util->remove_segment(topic.c_str());
    ASSERT_TRUE(ret_sec);
}

TEST(SHMTest, init_all_block) {
    boost::shared_ptr<sharedmem_transport::SharedMemoryUtil> 
        util(new sharedmem_transport::SharedMemoryUtil());
    sharedmem_transport::SharedMemoryBlock* descriptors_pub = NULL;
    std::string topic = "/chatter";
    int segment_size = 1000000;
    uint32_t queue_size = 10;
    uint64_t msg_size = 20;
    int32_t index = 1;

    util->remove_segment(topic.c_str());

    boost::interprocess::managed_shared_memory* segment = 
        util->create_segment(topic.c_str(), segment_size);
    ASSERT_TRUE(segment != NULL);

    sharedmem_transport::SharedMemorySegment* segment_mgr = util->create_segment_mgr(segment);
    ASSERT_TRUE(segment_mgr != NULL);

    uint8_t** addr_pub = new uint8_t*[queue_size];

    bool ret = segment_mgr->init_all_blocks(*segment, queue_size, 
        msg_size, descriptors_pub, addr_pub);
    ASSERT_TRUE(ret);

    bool ret_sec = util->remove_segment(topic.c_str());
    ASSERT_TRUE(ret_sec);

    sharedmem_transport::SharedMemoryPublisherImpl* shared_impl;
    bool ret_thd = shared_impl->create_topic_segement(topic, index, queue_size, msg_size, "", "", "");
    ASSERT_TRUE(ret_thd);

    if (addr_pub) {
        delete []addr_pub;
    }

    ASSERT_TRUE(util->remove_segment(topic.c_str()));
}

TEST(SHMTest, map_all_block) {
    boost::shared_ptr<sharedmem_transport::SharedMemoryUtil> 
        util(new sharedmem_transport::SharedMemoryUtil());
    std::string topic = "/chatter";

    util->remove_segment(topic.c_str());

    boost::interprocess::managed_shared_memory* segment = NULL;
    sharedmem_transport::SharedMemorySegment* segment_mgr = NULL;
    sharedmem_transport::SharedMemoryBlock* descriptors_sub = NULL;
    uint32_t queue_size = 10;

    bool ret = util->init_sharedmem(topic.c_str(), 
        segment, segment_mgr, descriptors_sub, queue_size);
    ASSERT_TRUE(ret);

    uint8_t** addr_sub = new uint8_t*[queue_size];
    bool ret_sec = segment_mgr->map_all_blocks(segment, queue_size, addr_sub);
    ASSERT_TRUE(ret_sec);

    if (addr_sub) {
        delete []addr_sub;
    }

    bool ret_thd = util->remove_segment(topic.c_str());
    ASSERT_TRUE(ret_thd);
}

TEST(SHMTest, lock_mutex) {
    boost::shared_ptr<sharedmem_transport::SharedMemoryUtil> 
        util(new sharedmem_transport::SharedMemoryUtil());
    std::string topic = "/chatter";

    util->remove_segment(topic.c_str());

    boost::shared_ptr<sharedmem_transport::SharedMemoryBlock> 
        block(new sharedmem_transport::SharedMemoryBlock());

    for (int i = 0; i < 10; ++i) {
        block->try_reserve_for_radical_write();
        usleep(1000);
        block->release_reserve_for_radical_write();
    }
    
    for (int i = 0; i < 10; ++i) {
        block->try_reserve_for_radical_read();
        usleep(1000);
        block->release_reserve_for_radical_read();
    } 

    ASSERT_TRUE(true);
}

TEST(SHMTest, write_msg) {
    boost::shared_ptr<sharedmem_transport::SharedMemoryUtil> 
        util(new sharedmem_transport::SharedMemoryUtil());
    sharedmem_transport::SharedMemoryBlock* descriptors_pub = NULL;
    std::string topic = "/chatter";
    int segment_size = 10;
    uint32_t queue_size = 10;
    int msg_size = 10;
    ros::SerializedMessage msg;

    util->remove_segment(topic.c_str());

    boost::interprocess::managed_shared_memory* segment = 
        util->create_segment(topic.c_str(), segment_size);
    ASSERT_TRUE(segment != NULL);

    sharedmem_transport::SharedMemorySegment* segment_mgr = util->create_segment_mgr(segment);
    ASSERT_TRUE(segment_mgr != NULL);

    uint8_t** addr_pub = new uint8_t*[queue_size];

    bool ret = segment_mgr->init_all_blocks(*segment, queue_size, 
        msg_size, descriptors_pub, addr_pub);
    ASSERT_TRUE(ret);

    for (int i = 0; i < 5; ++i) {
        bool ret_sec = segment_mgr->write_data(msg, queue_size, descriptors_pub, addr_pub);
        ASSERT_TRUE(ret_sec);
    }

    if (addr_pub) {
        delete []addr_pub;
    }

    bool ret_thd = util->remove_segment(topic.c_str());
    ASSERT_TRUE(ret_thd);
}

TEST(SHMTest, read_msg) {
    boost::shared_ptr<sharedmem_transport::SharedMemoryUtil> 
        util(new sharedmem_transport::SharedMemoryUtil());
    std::string topic = "/chatter";

    util->remove_segment(topic.c_str());
    
    boost::interprocess::managed_shared_memory* segment = NULL;
    sharedmem_transport::SharedMemorySegment* segment_mgr = NULL;
    sharedmem_transport::SharedMemoryBlock* descriptors_sub = NULL;

    uint32_t queue_size = 10;
    int32_t read_index = -1;
    int32_t msg_index;
    uint32_t msg_size;

    bool ret = util->init_sharedmem(topic.c_str(), segment, 
        segment_mgr, descriptors_sub, queue_size);
    ASSERT_TRUE(ret);

    uint8_t** addr_sub = new uint8_t*[queue_size];
    bool ret_sec = segment_mgr->map_all_blocks(segment, queue_size, addr_sub);
    ASSERT_TRUE(ret_sec);
    
    for (int i = 0; i < 100; ++i) {
        bool ret_thd = segment_mgr->read_data(read_index, 
            descriptors_sub, topic, msg_index, msg_size);
        EXPECT_EQ(false, ret_thd);
    }

    if (addr_sub) {
        delete []addr_sub;
    }

    ASSERT_TRUE(util->remove_segment(topic.c_str()));
}

}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::Time::init();
    return RUN_ALL_TESTS();
}