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
 * lightweight_logger_nodelet.cpp
 * Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <jsk_robot_startup/lightweight_logger.h>

namespace jsk_robot_startup
{
  namespace lifelog
  {
    void LightweightLogger::onInit()
    {
      initialized_ = false;
      jsk_topic_tools::StealthRelay::onInit();

      std::string db_name, col_name;
      nh_->param<std::string>("/robot/database", db_name, "jsk_robot_lifelog");
      nh_->param<std::string>("/robot/name", col_name, std::string());
      if (col_name.empty())
      {
        NODELET_FATAL_STREAM("Please specify param 'robot/name' (e.g. pr1012, olive)");
        return;
      }

      pnh_->param("wait_for_insert_", wait_for_insert_, false);

      NODELET_INFO_STREAM("Connecting to database " << db_name << "/" << col_name << "...");
      msg_store_.reset(new mongodb_store::MessageStoreProxy(*nh_, col_name, db_name));
      NODELET_INFO_STREAM("Successfully connected to database!");
      input_topic_name_ = pnh_->resolveName("input", true);
      initialized_ = true;
    }

    void LightweightLogger::inputCallback(const AnyMsgConstPtr& msg)
    {
      jsk_topic_tools::StealthRelay::inputCallback(msg);

      if (!initialized_) return;

      try
      {
        mongo::BSONObjBuilder meta;
        meta.append("input_topic", input_topic_name_);
        std::string doc_id = msg_store_->insert(*msg, meta.obj(), wait_for_insert_);
        if (doc_id.empty())
          NODELET_DEBUG_STREAM("Inserted (" << input_topic_name_ << ")");
        else
          NODELET_DEBUG_STREAM("Inserted (" << input_topic_name_ << "): " << doc_id);
      }
      catch (...) {
        NODELET_ERROR_STREAM("Failed to insert to db");
      }
    }
  }
}

#include <pluginlib/class_list_macros.h>
typedef jsk_robot_startup::lifelog::LightweightLogger LightweightLogger;
PLUGINLIB_EXPORT_CLASS(LightweightLogger, nodelet::Nodelet)
