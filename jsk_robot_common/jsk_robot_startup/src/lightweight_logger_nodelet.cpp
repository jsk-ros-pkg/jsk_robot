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

      // settings for database
      nh_->param<std::string>("/robot/database", db_name_, "jsk_robot_lifelog");
      nh_->param<std::string>("/robot/name", col_name_, std::string());
      if (col_name_.empty())
      {
        NODELET_FATAL_STREAM("Please specify param 'robot/name' (e.g. pr1012, olive)");
        return;
      }

      // settings for buffering logged messages
      int buffer_capacity;
      pnh_->param("buffer_capacity", buffer_capacity, 100);
      buffer_.set_capacity(buffer_capacity);

      // settings for blocking/non-blocking message insertion
      pnh_->param("wait_for_insert", wait_for_insert_, false);
      pnh_->param("vital_check", vital_check_, true);

      input_topic_name_ = pnh_->resolveName("input", true);

      // settings for diagnostics
      double vital_rate;
      pnh_->param("vital_rate", vital_rate, 1.0);
      vital_checker_.reset(
        new jsk_topic_tools::VitalChecker(1.0 / vital_rate));

      bool enable_diagnostics;
      pnh_->param<bool>("enable_diagnostics", enable_diagnostics, true);
      if (enable_diagnostics) {
        diagnostic_updater_.reset(
          new jsk_topic_tools::TimeredDiagnosticUpdater(*pnh_, ros::Duration(1.0)));
        diagnostic_updater_->setHardwareID("LightweightLogger");
        diagnostic_updater_->add(
          "LightweightLogger::" + input_topic_name_,
          boost::bind(&LightweightLogger::updateDiagnostic, this, _1));
        diagnostic_updater_->start();
      }

      inserted_count_ = 0;
      insert_error_count_ = 0;
      prev_insert_error_count_ = 0;
      init_time_ = ros::Time::now();

      // start logger thread
      logger_thread_ = boost::thread(
          boost::bind(&LightweightLogger::loggerThread, this));
    }

    LightweightLogger::~LightweightLogger() {
      NODELET_DEBUG_STREAM("destructor called");

      // stop logger thread
      if (logger_thread_.joinable()) {
        NODELET_DEBUG_STREAM("Shutting down logger thread");
        logger_thread_.interrupt();
        bool ok = logger_thread_.try_join_for(boost::chrono::seconds(5));
        if (ok) {
          NODELET_INFO_STREAM("successsfully stopped logger thread");
        } else {
          NODELET_WARN_STREAM("Timed out to join logger thread");
        }
      }

      // deinit message store object
      if (msg_store_) {
        msg_store_.reset();
      }
    }

    void LightweightLogger::loggerThread()
    {
      NODELET_DEBUG_STREAM("logger thread started");

      // The message store object is initialized here, since the object waits for connection
      // until the connection to the server is established.
      msg_store_.reset(new mongodb_store::MessageStoreProxy(*nh_, col_name_, db_name_));
      initialized_ = true;

      // After message store object is initialized, this thread is re-used for
      // lazy document insertion.
      bool being_interrupted = false;
      while (ros::ok()) {
        try {
          // check interruption
          if (being_interrupted) {
            if (!buffer_.empty()) {
              NODELET_WARN_STREAM(
                "The thread is interrupted through buffer is still not empty. "
                "Continue to insert");
            } else {
              NODELET_DEBUG_STREAM("buffer is empty. interrupted.");
              break;
            }
          }

          // lazy document insertion
          ros::MessageEvent<topic_tools::ShapeShifter const> event;
          const double timeout = 0.5;
          if (buffer_.get(event, timeout)) {
            const std::string& publisher_name = event.getPublisherName();
            const boost::shared_ptr<topic_tools::ShapeShifter const>& msg = event.getConstMessage();
            jsk_topic_tools::StealthRelay::inputCallback(msg);
            try
            {
              mongo::BSONObjBuilder meta;
              meta.append("input_topic", input_topic_name_);
              meta.append("published_by", publisher_name);
              std::string doc_id = msg_store_->insert(*msg, meta.obj(), true);
              NODELET_DEBUG_STREAM("Lazy inserted (" << input_topic_name_ << "): " << doc_id);
            }
            catch (...) {
              NODELET_ERROR_STREAM("Failed to lazy insert");
              if (being_interrupted) {
                NODELET_WARN_STREAM("Force exiting");
                break;
              }
            }
          } else {
            NODELET_DEBUG_STREAM("waiting for buffer...");
          }
        } catch (boost::thread_interrupted e) {
          NODELET_DEBUG_STREAM("logger thread being interrupted");
          being_interrupted = true;
        }
      }

      NODELET_DEBUG_STREAM("logger thread ended");
    }

    void LightweightLogger::inputCallback(const ros::MessageEvent<topic_tools::ShapeShifter const>& event)
    {
      const std::string& publisher_name = event.getPublisherName();
      const boost::shared_ptr<topic_tools::ShapeShifter const>& msg = event.getConstMessage();
      jsk_topic_tools::StealthRelay::inputCallback(msg);

      vital_checker_->poke();

      bool on_the_fly = initialized_ && buffer_.empty();
      if (!wait_for_insert_ && msg_store_->getNumInsertSubscribers() == 0) {
        // subscriber for message_store/insert does not exists
        on_the_fly = false;
      }

      if (on_the_fly) {
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
      } else {
        if (!initialized_) {
          NODELET_WARN_THROTTLE(1.0, "nodelet is not yet initialized");
        }
        if (buffer_.full()) {
          NODELET_WARN_THROTTLE(1.0, "buffer is full. discarded old elements");
        }
        buffer_.put(event);
        NODELET_DEBUG_STREAM("Put into buffer for lazy insertion");
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
      } else if (vital_check_ && !vital_checker_->isAlive()) {
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
