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

#ifndef ACTIONLIB_ACTION_CLIENT_H_
#define ACTIONLIB_ACTION_CLIENT_H_

#include <boost/thread/condition.hpp>

#include "ros/ros.h"
#include "ros/callback_queue_interface.h"
#include <actionlib/client/client_helpers.h>
#include <actionlib/client/connection_monitor.h>
#include <actionlib/destruction_guard.h>

namespace actionlib
{

/**
 * \brief Full interface to an ActionServer
 *
 * ActionClient provides a complete client side implementation of the ActionInterface protocol.
 * It provides callbacks for every client side transition, giving the user full observation into
 * the client side state machine.
 */
template <class ActionSpec>
class ActionClient
{
public:
  typedef ClientGoalHandle<ActionSpec> GoalHandle;

private:
  ACTION_DEFINITION(ActionSpec);
  typedef ActionClient<ActionSpec> ActionClientT;
  typedef boost::function<void (GoalHandle) > TransitionCallback;
  typedef boost::function<void (GoalHandle, const FeedbackConstPtr&) > FeedbackCallback;
  typedef boost::function<void (const ActionGoalConstPtr)> SendGoalFunc;

public:
  /**
   * \brief Simple constructor
   *
   * Constructs an ActionClient and sets up the necessary ros topics for the ActionInterface
   * \param name The action name. Defines the namespace in which the action communicates
   * \param queue CallbackQueue from which this action will process messages.
   *              The default (NULL) is to use the global queue
   */
 ActionClient(const std::string& name, ros::CallbackQueueInterface* queue = NULL)
  : n_(name), guard_(new DestructionGuard()),
    manager_(guard_)
  {
    initClient(queue);
  }

  /**
   * \brief Constructor with namespacing options
   *
   * Constructs an ActionClient and sets up the necessary ros topics for the ActionInterface,
   * and namespaces them according the a specified NodeHandle
   * \param n The node handle on top of which we want to namespace our action
   * \param name The action name. Defines the namespace in which the action communicates
   * \param queue CallbackQueue from which this action will process messages.
   *              The default (NULL) is to use the global queue
   */
  ActionClient(const ros::NodeHandle& n, const std::string& name, ros::CallbackQueueInterface* queue = NULL)
    : n_(n, name), guard_(new DestructionGuard()),
    manager_(guard_)
  {
    initClient(queue);
  }

  ~ActionClient()
  {
    ROS_DEBUG_NAMED("actionlib", "ActionClient: Waiting for destruction guard to clean up");
    guard_->destruct();
    ROS_DEBUG_NAMED("actionlib", "ActionClient: destruction guard destruct() done");
  }


  /**
   * \brief Sends a goal to the ActionServer, and also registers callbacks
   * \param transition_cb Callback that gets called on every client state transition
   * \param feedback_cb Callback that gets called whenever feedback for this goal is received
   */
  GoalHandle sendGoal(const Goal& goal,
                      TransitionCallback transition_cb = TransitionCallback(),
                      FeedbackCallback   feedback_cb   = FeedbackCallback())
  {
    ROS_DEBUG_NAMED("actionlib", "about to start initGoal()");
    GoalHandle gh = manager_.initGoal(goal, transition_cb, feedback_cb);
    ROS_DEBUG_NAMED("actionlib", "Done with initGoal()");

    return gh;
  }

  /**
   * \brief Cancel all goals currently running on the action server
   *
   * This preempts all goals running on the action server at the point that
   * this message is serviced by the ActionServer.
   */
  void cancelAllGoals()
  {
    actionlib_msgs::GoalID cancel_msg;
    // CancelAll policy encoded by stamp=0, id=0
    cancel_msg.stamp = ros::Time(0,0);
    cancel_msg.id = "";
    cancel_pub_.publish(cancel_msg);
  }

  /**
   * \brief Cancel all goals that were stamped at and before the specified time
   * \param time All goals stamped at or before `time` will be canceled
   */
  void cancelGoalsAtAndBeforeTime(const ros::Time& time)
  {
    actionlib_msgs::GoalID cancel_msg;
    cancel_msg.stamp = time;
    cancel_msg.id = "";
    cancel_pub_.publish(cancel_msg);
  }

  /**
   * \brief Waits for the ActionServer to connect to this client
   * Often, it can take a second for the action server & client to negotiate
   * a connection, thus, risking the first few goals to be dropped. This call lets
   * the user wait until the network connection to the server is negotiated
   * NOTE: Using this call in a single threaded ROS application, or any
   * application where the action client's callback queue is not being
   * serviced, will not work. Without a separate thread servicing the queue, or
   * a multi-threaded spinner, there is no way for the client to tell whether
   * or not the server is up because it can't receive a status message.
   * \param timeout Max time to block before returning. A zero timeout is interpreted as an infinite timeout.
   * \return True if the server connected in the allocated time. False on timeout
   */
  bool waitForActionServerToStart(const ros::Duration& timeout = ros::Duration(0,0) )
  {
    if (connection_monitor_)
      return connection_monitor_->waitForActionServerToStart(timeout, n_);
    else
      return false;
  }

  /**
   * @brief  Checks if the action client is successfully connected to the action server
   * @return True if the server is connected, false otherwise
   */
  bool isServerConnected()
  {
    return connection_monitor_->isServerConnected();
  }

private:
  ros::NodeHandle n_;

  boost::shared_ptr<DestructionGuard> guard_;
  GoalManager<ActionSpec> manager_;

  ros::Subscriber result_sub_;
  ros::Subscriber feedback_sub_;

  boost::shared_ptr<ConnectionMonitor> connection_monitor_;   // Have to destroy subscribers and publishers before the connection_monitor_, since we call callbacks in the connection_monitor_

  ros::Publisher  goal_pub_;
  ros::Publisher  cancel_pub_;
  ros::Subscriber status_sub_;

  void sendGoalFunc(const ActionGoalConstPtr& action_goal)
  {
    goal_pub_.publish(action_goal);
  }

  void sendCancelFunc(const actionlib_msgs::GoalID& cancel_msg)
  {
    cancel_pub_.publish(cancel_msg);
  }

  void initClient(ros::CallbackQueueInterface* queue)
  {
    status_sub_   = queue_subscribe("status",   1, &ActionClientT::statusCb,   this, queue);
    feedback_sub_ = queue_subscribe("feedback", 1, &ActionClientT::feedbackCb, this, queue);
    result_sub_   = queue_subscribe("result",   1, &ActionClientT::resultCb,   this, queue);

    connection_monitor_.reset(new ConnectionMonitor(feedback_sub_, status_sub_));

    // Start publishers and subscribers
    goal_pub_ = queue_advertise<ActionGoal>("goal", 10,
                                            boost::bind(&ConnectionMonitor::goalConnectCallback,    connection_monitor_, _1),
                                            boost::bind(&ConnectionMonitor::goalDisconnectCallback, connection_monitor_, _1),
                                            queue);
    cancel_pub_ = queue_advertise<actionlib_msgs::GoalID>("cancel", 10,
                                            boost::bind(&ConnectionMonitor::cancelConnectCallback,    connection_monitor_, _1),
                                            boost::bind(&ConnectionMonitor::cancelDisconnectCallback, connection_monitor_, _1),
                                            queue);

    manager_.registerSendGoalFunc(boost::bind(&ActionClientT::sendGoalFunc, this, _1));
    manager_.registerCancelFunc(boost::bind(&ActionClientT::sendCancelFunc, this, _1));

  }

  template <class M>
  ros::Publisher queue_advertise(const std::string& topic, uint32_t queue_size,
                                 const ros::SubscriberStatusCallback& connect_cb,
                                 const ros::SubscriberStatusCallback& disconnect_cb,
                                 ros::CallbackQueueInterface* queue)
  {
    ros::AdvertiseOptions ops;
    ops.init<M>(topic, queue_size, connect_cb, disconnect_cb);
    ops.tracked_object = ros::VoidPtr();
    ops.latch = false;
    ops.callback_queue = queue;
    return n_.advertise(ops);
  }

  template<class M, class T>
  ros::Subscriber queue_subscribe(const std::string& topic, uint32_t queue_size, void(T::*fp)(const ros::MessageEvent<M const>&), T* obj, ros::CallbackQueueInterface* queue)
  {
    ros::SubscribeOptions ops;
    ops.callback_queue = queue;
    ops.topic = topic;
    ops.queue_size = queue_size;
    ops.md5sum = ros::message_traits::md5sum<M>();
    ops.datatype = ros::message_traits::datatype<M>();
    ops.helper = ros::SubscriptionCallbackHelperPtr(
      new ros::SubscriptionCallbackHelperT<const ros::MessageEvent<M const>& >(
        boost::bind(fp, obj, _1)
      )
    );
    return n_.subscribe(ops);
  }

  void statusCb(const ros::MessageEvent<actionlib_msgs::GoalStatusArray const>& status_array_event)
  {
    ROS_DEBUG_NAMED("actionlib", "Getting status over the wire.");
    if (connection_monitor_)
      connection_monitor_->processStatus(status_array_event.getConstMessage(), status_array_event.getPublisherName());
    manager_.updateStatuses(status_array_event.getConstMessage());
  }

  void feedbackCb(const ros::MessageEvent<ActionFeedback const>& action_feedback)
  {
    manager_.updateFeedbacks(action_feedback.getConstMessage());
  }

  void resultCb(const ros::MessageEvent<ActionResult const>& action_result)
  {
    manager_.updateResults(action_result.getConstMessage());
  }
};


}

#endif
