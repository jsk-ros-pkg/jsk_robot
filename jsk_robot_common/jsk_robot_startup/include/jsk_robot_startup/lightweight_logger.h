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
 * lightweight_logger.h
 * Author: Furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */


#ifndef LIGHTWEIGHT_LOGGER_H__
#define LIGHTWEIGHT_LOGGER_H__

#include <boost/circular_buffer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <topic_tools/shape_shifter.h>
#include <jsk_topic_tools/diagnostic_utils.h>
#include <jsk_topic_tools/stealth_relay.h>
#include <jsk_topic_tools/timered_diagnostic_updater.h>
#include <jsk_topic_tools/vital_checker.h>
#include <mongodb_store/message_store.h>

namespace jsk_robot_startup
{
  namespace lifelog
  {

    template <typename T>
    class blocking_circular_buffer {
      /* Thread safe blocking circular buffer */
    public:
      blocking_circular_buffer() {}

      void put(const T& value) {
        boost::mutex::scoped_lock lock(mutex_);
        const bool prev_empty = empty();
        buffer_.push_back(value);
        if (prev_empty) empty_wait_.notify_all();
      }

      bool get(T& value, double timeout=0.0) {
        boost::posix_time::ptime start = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration elapsed;
        while (ros::ok()) {
          elapsed = boost::posix_time::microsec_clock::local_time() - start;
          if (timeout > 0 &&
              (double)(elapsed.total_milliseconds() / 1000.0) > timeout)
            break;
          boost::mutex::scoped_lock lock(mutex_);
          if (empty()) empty_wait_.wait(lock);
          if (!empty()) {
            value = buffer_.front();
            buffer_.pop_front();
            return true;
          }
        }
        return false;
      }

      void clear() {
        buffer_.clear();
      }

      const bool empty() const {
        return buffer_.empty();
      }

      const bool full() const {
        return buffer_.full();
      }

      const int size() const {
        return buffer_.size();
      }

      void set_capacity(int cap) {
        buffer_.set_capacity(cap);
      }
    protected:
      boost::circular_buffer<T> buffer_;
      boost::condition_variable empty_wait_;
      boost::mutex mutex_;
    };
    
    class LightweightLogger : public jsk_topic_tools::StealthRelay
    {
    protected:
      virtual void onInit();
      virtual ~LightweightLogger();
      virtual void loggerThread();
      virtual void inputCallback(const ros::MessageEvent<topic_tools::ShapeShifter const>& event);
      virtual void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);

      boost::shared_ptr<mongodb_store::MessageStoreProxy> msg_store_;
      boost::thread logger_thread_;
      bool wait_for_insert_;
      bool vital_check_;
      bool initialized_;
      std::string input_topic_name_;
      std::string db_name_, col_name_;
      blocking_circular_buffer<ros::MessageEvent<topic_tools::ShapeShifter const> > buffer_;

      // diagnostics
      ros::Time init_time_;
      uint64_t inserted_count_, insert_error_count_, prev_insert_error_count_;
      jsk_topic_tools::VitalChecker::Ptr vital_checker_;
      jsk_topic_tools::TimeredDiagnosticUpdater::Ptr diagnostic_updater_;
    };
  }
}

#endif // LIGHTWEIGHT_LOGGER_H__
