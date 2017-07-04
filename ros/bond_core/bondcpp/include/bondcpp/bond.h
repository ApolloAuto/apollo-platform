/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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

/** \author Stuart Glaser */

#ifndef BONDCPP_BOND_H
#define BONDCPP_BOND_H

#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <ros/ros.h>

#include <bondcpp/timeout.h>
#include <bond/Constants.h>
#include <bond/Status.h>
#include "BondSM_sm.h"

namespace bond {

/** \brief Forms a bond to monitor another process.
 *
 * The bond::Bond class implements a bond, allowing you to monitor
 * another process and be notified when it dies.  In turn, it will be
 * notified when you die.
 */
class Bond
{
public:
  /** \brief Constructs a bond, but does not connect
   *
   * \param topic The topic used to exchange the bond status messages.
   * \param id The ID of the bond, which should match the ID used on
   *           the sister's end.
   * \param on_broken callback that will be called when the bond is broken.
   * \param on_formed callback that will be called when the bond is formed.
   */
  Bond(const std::string &topic, const std::string &id,
       boost::function<void(void)> on_broken = boost::function<void(void)>(),
       boost::function<void(void)> on_formed = boost::function<void(void)>());

  /** \brief Destructs the object, breaking the bond if it is still formed.
   */
  ~Bond();

  double getConnectTimeout() const { return connect_timeout_; }
  void setConnectTimeout(double dur);
  double getDisconnectTimeout() const { return disconnect_timeout_; }
  void setDisconnectTimeout(double dur);
  double getHeartbeatTimeout() const { return heartbeat_timeout_; }
  void setHeartbeatTimeout(double dur);
  double getHeartbeatPeriod() const { return heartbeat_period_; }
  void setHeartbeatPeriod(double dur);

  void setCallbackQueue(ros::CallbackQueueInterface *queue);

  /** \brief Starts the bond and connects to the sister process.
   */
  void start();

  /** \brief Sets the formed callback.
   */
  void setFormedCallback(boost::function<void(void)> on_formed);

  /** \brief Sets the broken callback
   */
  void setBrokenCallback(boost::function<void(void)> on_broken);

  /** \brief Blocks until the bond is formed for at most 'duration'.
   *
   * \param timeout Maximum duration to wait.  If -1 then this call will not timeout.
   * \return true iff the bond has been formed.
   */
  bool waitUntilFormed(ros::Duration timeout = ros::Duration(-1));

  /** \brief Blocks until the bond is formed for at most 'duration'.
   *
   * \param timeout Maximum duration to wait.  If -1 then this call will not timeout.
   * \return true iff the bond has been formed.
   */
  bool waitUntilFormed(ros::WallDuration timeout = ros::WallDuration(-1));

  /** \brief Blocks until the bond is broken for at most 'duration'.
   *
   * \param timeout Maximum duration to wait.  If -1 then this call will not timeout.
   * \return true iff the bond has been broken, even if it has never been formed.
   */
  bool waitUntilBroken(ros::Duration timeout = ros::Duration(-1));

  /** \brief Blocks until the bond is broken for at most 'duration'.
   *
   * \param timeout Maximum duration to wait.  If -1 then this call will not timeout.
   * \return true iff the bond has been broken, even if it has never been formed.
   */
  bool waitUntilBroken(ros::WallDuration timeout = ros::WallDuration(-1));

  /** \brief Indicates if the bond is broken
   * \return true iff the bond has been broken.
   */
  bool isBroken();

  /** \brief Breaks the bond, notifying the other process.
   */
  void breakBond();

  std::string getTopic() { return topic_; }
  std::string getId() { return id_; }
  std::string getInstanceId() { return instance_id_; }

private:
  friend class ::BondSM;

  ros::NodeHandle nh_;
  boost::scoped_ptr<BondSM> bondsm_;
  BondSMContext sm_;

  std::string topic_;
  std::string id_;
  std::string instance_id_;
  std::string sister_instance_id_;
  boost::function<void(void)> on_broken_;
  boost::function<void(void)> on_formed_;
  bool sisterDiedFirst_;
  bool started_;

  boost::mutex mutex_;
  boost::condition condition_;

  double connect_timeout_;
  double heartbeat_timeout_;
  double disconnect_timeout_;
  double heartbeat_period_;

  Timeout connect_timer_;
  Timeout heartbeat_timer_;
  Timeout disconnect_timer_;

  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::WallTimer publishingTimer_;

  void onConnectTimeout();
  void onHeartbeatTimeout();
  void onDisconnectTimeout();

  void bondStatusCB(const bond::Status::ConstPtr &msg);

  void doPublishing(const ros::WallTimerEvent &e);
  void publishStatus(bool active);

  std::vector<boost::function<void(void)> > pending_callbacks_;
  void flushPendingCallbacks();
};

}// namespace


// Internal use only
struct BondSM
{
  BondSM(bond::Bond *b_) : b(b_) {}
  void Connected();
  void SisterDied();
  void Death();
  void Heartbeat();
  void StartDying();

private:
  bond::Bond *b;
};

#endif
