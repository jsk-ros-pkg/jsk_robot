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

      nh_->param<std::string>("/robot/database", db_name_, "jsk_robot_lifelog");
      nh_->param<std::string>("/robot/name", col_name_, std::string());
      if (col_name_.empty())
      {
        NODELET_FATAL_STREAM("Please specify param 'robot/name' (e.g. pr1012, olive)");
        return;
      }

      pnh_->param("wait_for_insert", wait_for_insert_, false);

      input_topic_name_ = pnh_->resolveName("input", true);

      diagnostic_updater_.reset(
        new jsk_topic_tools::TimeredDiagnosticUpdater(*pnh_, ros::Duration(1.0)));
      diagnostic_updater_->setHardwareID("LightweightLogger");
      diagnostic_updater_->add(
        "LightweightLogger::" + input_topic_name_,
        boost::bind(&LightweightLogger::updateDiagnostic, this, _1));

      double vital_rate;
      pnh_->param("vital_rate", vital_rate, 1.0);
      vital_checker_.reset(
        new jsk_topic_tools::VitalChecker(1.0 / vital_rate));

      diagnostic_updater_->start();

      inserted_count_ = 0;
      insert_error_count_ = 0;
      prev_insert_error_count_ = 0;
      init_time_ = ros::Time::now();

      deferred_load_thread_ = boost::thread(
          boost::bind(&LightweightLogger::loadThread, this));
    }

    LightweightLogger::~LightweightLogger() {
      if (!initialized_) {
        NODELET_DEBUG_STREAM("Shutting down deferred load thread");
        deferred_load_thread_.join();
        NODELET_DEBUG_STREAM("deferred load thread stopped");
      }
    }

    void LightweightLogger::loadThread()
    {
      NODELET_INFO_STREAM("Connecting to database " << db_name_ << "/" << col_name_ << "...");
      msg_store_ = MessageStoreSingleton::getInstance(col_name_, db_name_);
      NODELET_INFO_STREAM("Successfully connected to database!");

      initialized_ = true;
    }

    void LightweightLogger::inputCallback(const ros::MessageEvent<topic_tools::ShapeShifter>& event)
    {
      const std::string& publisher_name = event.getPublisherName();
      const boost::shared_ptr<topic_tools::ShapeShifter const>& msg = event.getConstMessage();
      jsk_topic_tools::StealthRelay::inputCallback(msg);

      if (!initialized_) return;
      vital_checker_->poke();

      try
      {
        mongo::BSONObjBuilder meta;
        meta.append("input_topic", input_topic_name_);
        meta.append("published_by", publisher_name);
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

    void LightweightLogger::updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
      if (ros::Time::now() - init_time_ < ros::Duration(10.0)) {
        if (initialized_) {
          stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                       getName() + " initialized");
        } else {
          stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                       getName() + " is in initialization");
        }
      } else if (!initialized_) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                     getName() + " is taking too long to be initialized");
      } else if (!vital_checker_->isAlive()) {
        jsk_topic_tools::addDiagnosticErrorSummary(getName(), vital_checker_, stat);
      } else if (insert_error_count_ != prev_insert_error_count_) {
        stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                     (boost::format("%s fails to insert to db for %d times") % getName() % insert_error_count_).str());
        prev_insert_error_count_ = insert_error_count_;
      } else {
        stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                     getName() + " is running");
      }

      stat.add("Inserted", inserted_count_);
      stat.add("Insert Failure", insert_error_count_);
      vital_checker_->registerStatInfo(stat, "Last Insert");
    }
  } // lifelog
} // jsk_robot_startup

#include <pluginlib/class_list_macros.h>
typedef jsk_robot_startup::lifelog::LightweightLogger LightweightLogger;
PLUGINLIB_EXPORT_CLASS(LightweightLogger, nodelet::Nodelet)
