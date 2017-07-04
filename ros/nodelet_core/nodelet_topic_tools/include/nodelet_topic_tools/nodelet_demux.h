/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *  COPYRIGHT OWNERff OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: nodelet_mux.h 27892 2010-02-25 06:47:29Z rusu $
 *
 */
#ifndef NODELET_NODELET_DEMUX_H_
#define NODELET_NODELET_DEMUX_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>

namespace nodelet
{
  /** \brief @b NodeletDEMUX represent a demux nodelet for topics: it takes 1 input topic, and publishes on N (<=8) output topics.
    * \author Radu Bogdan Rusu
    */
  template <typename T, typename Subscriber = message_filters::Subscriber<T> >
  class NodeletDEMUX: public Nodelet
  {
    typedef typename boost::shared_ptr<const T> TConstPtr;
    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Nodelet initialization routine. */
      void
        onInit ()
      {
        private_nh_ = getPrivateNodeHandle ();
        sub_input_.subscribe (private_nh_, "input", 1, bind (&NodeletDEMUX<T,Subscriber>::input_callback, this, _1));

        if (!private_nh_.getParam ("output_topics", output_topics_))
        {
          ROS_ERROR ("[nodelet::NodeletDEMUX::init] Need a 'output_topics' parameter to be set before continuing!");
          return; 
        }
        // Check the type
        switch (output_topics_.getType ())
        {
          case XmlRpc::XmlRpcValue::TypeArray:
          {
            if (output_topics_.size () == 1)
            {
              ROS_ERROR ("[nodelet::NodeletDEMUX::init] Only one topic given. Does it make sense to passthrough?");
              return;
            }

            if (output_topics_.size () > 8)
            {
              ROS_ERROR ("[nodelet::NodeletDEMUX::init] More than 8 topics passed!");
              return;
             }

            ROS_INFO_STREAM ("[nodelet::NodeletDEMUX::init] Publishing to " << output_topics_.size () << " user given topics as outputs:");
            for (int d = 0; d < output_topics_.size (); ++d)
              ROS_INFO_STREAM (" - " << (std::string)(output_topics_[d]));

            pubs_output_.resize (output_topics_.size ());
            for (int d = 0; d < output_topics_.size (); ++d)
              *pubs_output_[d] = private_nh_.template advertise<T> ((std::string)(output_topics_[d]), 1);
            break;
          }
          default:
          {
            ROS_ERROR ("[nodelet::NodeletDEMUX::init] Invalid 'output_topics' parameter given!");
            return;
          }
        }
      }

    private:

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void
        input_callback (const TConstPtr &input)
      {
        for (size_t d = 0; d < pubs_output_.size (); ++d)
          pubs_output_[d]->publish (input);
      }

      /** \brief ROS local node handle. */
      ros::NodeHandle private_nh_;
      /** \brief The output list of publishers. */
      std::vector<boost::shared_ptr <ros::Publisher> > pubs_output_;
      /** \brief The input subscriber. */
      Subscriber sub_input_;
            

      /** \brief The list of output topics passed as a parameter. */
      XmlRpc::XmlRpcValue output_topics_;
  };

  /** \brief @b NodeletDEMUX represent a demux nodelet for topics: it takes 1 input topic, and publishes on N (<=8) output topics.
    * \author Radu Bogdan Rusu
    */
  template <typename T>
  class NodeletDEMUX<T, message_filters::Subscriber<T> >: public Nodelet
  {
    typedef typename boost::shared_ptr<const T> TConstPtr;
    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Nodelet initialization routine. */
      void
        onInit ()
      {
        private_nh_ = getPrivateNodeHandle ();

        sub_input_ = private_nh_.subscribe ("input", 1, &NodeletDEMUX<T>::input_callback, this);

        if (!private_nh_.getParam ("output_topics", output_topics_))
        {
          ROS_ERROR ("[nodelet::NodeletDEMUX::init] Need a 'output_topics' parameter to be set before continuing!");
          return; 
        }
        // Check the type
        switch (output_topics_.getType ())
        {
          case XmlRpc::XmlRpcValue::TypeArray:
          {
            if (output_topics_.size () == 1)
            {
              ROS_ERROR ("[nodelet::NodeletDEMUX::init] Only one topic given. Does it make sense to passthrough?");
              return;
            }

            if (output_topics_.size () > 8)
            {
              ROS_ERROR ("[nodelet::NodeletDEMUX::init] More than 8 topics passed!");
              return;
             }

            ROS_INFO_STREAM ("[nodelet::NodeletDEMUX::init] Publishing to " << output_topics_.size () << " user given topics as outputs:");
            for (int d = 0; d < output_topics_.size (); ++d)
              ROS_INFO_STREAM (" - " << (std::string)(output_topics_[d]));

            pubs_output_.resize (output_topics_.size ());
            for (int d = 0; d < output_topics_.size (); ++d)
              *pubs_output_[d] = private_nh_.template advertise<T> ((std::string)(output_topics_[d]), 1);
            break;
          }
          default:
          {
            ROS_ERROR ("[nodelet::NodeletDEMUX::init] Invalid 'output_topics' parameter given!");
            return;
          }
        }
      }

    private:

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      void
        input_callback (const TConstPtr &input)
      {
        for (size_t d = 0; d < pubs_output_.size (); ++d)
          pubs_output_[d]->publish (input);
      }

      /** \brief ROS local node handle. */
      ros::NodeHandle private_nh_;
      /** \brief The output list of publishers. */
      std::vector<boost::shared_ptr <ros::Publisher> > pubs_output_;
      /** \brief The input subscriber. */
      ros::Subscriber sub_input_;
            

      /** \brief The list of output topics passed as a parameter. */
      XmlRpc::XmlRpcValue output_topics_;
  };

}
#endif
