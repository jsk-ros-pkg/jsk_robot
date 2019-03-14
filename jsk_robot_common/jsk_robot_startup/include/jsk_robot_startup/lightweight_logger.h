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

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <topic_tools/shape_shifter.h>
#include <jsk_topic_tools/diagnostic_utils.h>
#include <jsk_topic_tools/stealth_relay.h>
#include <jsk_topic_tools/timered_diagnostic_updater.h>
#include <jsk_topic_tools/vital_checker.h>
#include <jsk_robot_startup/message_store_singleton.h>

namespace jsk_robot_startup
{
  namespace lifelog
  {
    class LightweightLogger : public jsk_topic_tools::StealthRelay
    {
    protected:
      virtual void onInit();
      virtual ~LightweightLogger();
      virtual void loadThread();
      virtual void inputCallback(const ros::MessageEvent<topic_tools::ShapeShifter>& event);
      virtual void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);

      mongodb_store::MessageStoreProxy* msg_store_;
      boost::thread deferred_load_thread_;
      bool wait_for_insert_;
      bool initialized_;
      std::string input_topic_name_;
      std::string db_name_, col_name_;

      // diagnostics
      ros::Time init_time_;
      uint64_t inserted_count_, insert_error_count_, prev_insert_error_count_;
      jsk_topic_tools::VitalChecker::Ptr vital_checker_;
      jsk_topic_tools::TimeredDiagnosticUpdater::Ptr diagnostic_updater_;
    };
  }
}

#endif // LIGHTWEIGHT_LOGGER_H__
