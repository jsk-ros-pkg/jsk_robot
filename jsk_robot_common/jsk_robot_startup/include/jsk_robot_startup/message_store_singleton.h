// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
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
/*
 * message_store_singleton.h
 * Author:  <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */


#ifndef MESSAGE_STORE_SINGLETON_H__
#define MESSAGE_STORE_SINGLETON_H__

#include <boost/thread.hpp>
#include <mongodb_store/message_store.h>


namespace jsk_robot_startup
{
namespace lifelog
{
class MessageStoreSingleton
{
 public:
  typedef std::map<std::string, mongodb_store::MessageStoreProxy*> M_Proxy;
  static mongodb_store::MessageStoreProxy* getInstance(
      const std::string& collection="message_store",
      const std::string& database="message_store",
      const std::string& prefix="/message_store");
  static void destroy();
 protected:
  static ros::NodeHandle nh_;
  static M_Proxy instances_;
  static boost::mutex mutex_;
 private:
  // prohibit constructor
  MessageStoreSingleton(MessageStoreSingleton const&) {};
  MessageStoreSingleton& operator=(MessageStoreSingleton const&) {};
};
} // lifelog
} // jsk_robot_startup

#endif // MESSAGE_STORE_SINGLETON_H__
