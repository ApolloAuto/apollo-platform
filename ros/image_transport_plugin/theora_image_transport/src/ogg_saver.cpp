/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 20012, Willow Garage, Inc.
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

#include <ros/ros.h>

#include <theora_image_transport/Packet.h>

#include <theora/codec.h>
#include <theora/theoraenc.h>
#include <theora/theoradec.h>
#include <ogg/ogg.h>

#include <fstream>
#include <vector>
#include <boost/scoped_array.hpp>

using namespace std;

class OggSaver
{
public:
  OggSaver(const char* filename)
   : fout_(filename, std::ios::out|std::ios::binary)
  {
    if (ogg_stream_init(&stream_state_, 0) == -1) {
      ROS_FATAL("Unable to initialize ogg_stream_state structure");
      exit(1);
    }

    sub_ = nh_.subscribe("stream", 10, &OggSaver::processMsg, this);
  }

  ~OggSaver()
  {
    ogg_page page;
    if (ogg_stream_flush(&stream_state_, &page) != 0)
      writePage(page);
    fout_.close();
    ogg_stream_clear(&stream_state_);
  }

private:

  ros::NodeHandle nh_;
  ogg_stream_state stream_state_;
  ofstream fout_;
  ros::Subscriber sub_;

  // When using this caller is responsible for deleting oggpacket.packet!!
  void msgToOggPacket(const theora_image_transport::Packet &msg, ogg_packet &oggpacket)
  {
    oggpacket.bytes = msg.data.size();
    oggpacket.b_o_s = msg.b_o_s;
    oggpacket.e_o_s = msg.e_o_s;
    oggpacket.granulepos = msg.granulepos;
    oggpacket.packetno = msg.packetno;
    oggpacket.packet = new unsigned char[oggpacket.bytes];
    memcpy(oggpacket.packet, &msg.data[0], oggpacket.bytes);
  }

  void writePage(ogg_page& page)
  {
    fout_.write((char*)page.header, page.header_len);
    fout_.write((char*)page.body,   page.body_len);
  }

  void processMsg(const theora_image_transport::PacketConstPtr& message)
  {
    /// @todo Make sure we don't write a video packet first
    /// @todo Handle duplicate headers
    /// @todo Wait for a keyframe!!
    /// @todo Need to flush page for initial identification header packet? And after last header packet?
    /// @todo Handle chaining streams? Need to retroactively set e_o_s on previous video packet.
    ogg_packet oggpacket;
    msgToOggPacket(*message, oggpacket);
    boost::scoped_array<unsigned char> packet_guard(oggpacket.packet); // Make sure packet memory gets deleted

    if (ogg_stream_packetin(&stream_state_, &oggpacket)) {
      ROS_ERROR("Error while adding packet to stream.");
      exit(2);
    }

    ogg_page page;
    if (ogg_stream_pageout(&stream_state_, &page) != 0)
      writePage(page);
  }
};

int main(int argc, char** argv)
{
  /// @todo Use image topic, not stream
  /// @todo Option to specify (or figure out?) the frame rate
  ros::init(argc, argv, "OggSaver", ros::init_options::AnonymousName);

  if(argc < 2) {
    cerr << "Usage: " << argv[0] << " stream:=/theora/image/stream outputFile" << endl;
    exit(3);
  }
  if (ros::names::remap("stream") == "stream") {
      ROS_WARN("ogg_saver: stream has not been remapped! Typical command-line usage:\n"
               "\t$ ./ogg_saver stream:=<theora stream topic> outputFile");
  }
  
  OggSaver saver(argv[1]);
  
  ros::spin();
  return 0;
}
