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
 * joint_states_throttle_node.cpp
 * Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */


#include <algorithm>
#include <cmath>
#include <angles/angles.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_robot_startup/JointStatesThrottleConfig.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>


namespace jsk_robot_startup {
  namespace lifelog {
    class JointStatesThrottle {
    public:
      typedef jsk_robot_startup::JointStatesThrottleConfig Config;

#if URDFDOM_1_0_0_API
      typedef urdf::JointSharedPtr JointSharedPtr;
#else
      typedef boost::shared_ptr<urdf::Joint> JointSharedPtr;
#endif
      JointStatesThrottle() : nh_(), pnh_("~") {

        if (!robot_model_.initParam("robot_description")) {
          ROS_ERROR_STREAM("Failed to load robot_desription");
          return;
        }

        nh_.param<bool>("use_sim_time", use_sim_time_, false);
        pnh_.param<std::vector<std::string> >("blacklist", blacklist_, std::vector<std::string>());

        if (!blacklist_.empty()) {
          ROS_INFO_STREAM(blacklist_.size() << " blacklist joints registered");
        }

        srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(pnh_);
        dynamic_reconfigure::Server<Config>::CallbackType f =
          boost::bind(&JointStatesThrottle::configCallback, this, _1, _2);
        srv_->setCallback(f);

        pub_ = pnh_.advertise<sensor_msgs::JointState>("output", 100);
        sub_ = pnh_.subscribe("input", 1, &JointStatesThrottle::jointStatesCallback, this);
      }

      ~JointStatesThrottle() {}

      void configCallback(Config& config, uint32_t level) {
        boost::mutex::scoped_lock lock(mutex_);
        periodic_ = config.periodic;
        periodic_rate_ = config.periodic_rate;
        threshold_ = config.threshold;
        rotation_factor_ = config.rotation_factor;
      }

      void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
        boost::mutex::scoped_lock lock(mutex_);

        double diff = 0.0;

        if (msg->name.size() != msg->position.size()) return;

        if (last_msg_.name.empty()) {
          last_msg_ = *msg;
          return;
        }

        if (use_sim_time_ && msg->header.stamp < last_msg_.header.stamp) {
          ROS_WARN_STREAM("Time jumped back.");
          last_msg_ = *msg;
          return;
        }

        if (periodic_) {
          double diff_time = (msg->header.stamp - last_msg_.header.stamp).toSec();
          if (diff_time > periodic_rate_) {
            pub_.publish(*msg);
            last_msg_ = *msg;
          }
          return;
        }

        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (std::find(blacklist_.begin(), blacklist_.end(), msg->name[i]) !=
              blacklist_.end()) {
            continue; // the joint is on blacklist
          }
          std::vector<std::string>::iterator lit = std::find(
            last_msg_.name.begin(), last_msg_.name.end(), msg->name[i]);
          if (lit == last_msg_.name.end()) {
            continue; // the joint is not on the last message
          }
          size_t j = std::distance(last_msg_.name.begin(), lit);

          std::map<std::string, JointSharedPtr >::iterator jit =
            robot_model_.joints_.find(msg->name[i]);
          if (jit == robot_model_.joints_.end()) {
            ROS_ERROR_STREAM_ONCE(msg->name[i] << " not found in 'robot_description'");
            continue;
          }

          if (jit->second->type == urdf::Joint::REVOLUTE ||
              jit->second->type == urdf::Joint::CONTINUOUS) {
            diff += std::abs(angles::shortest_angular_distance(
                                  msg->position[i], last_msg_.position[j])) * rotation_factor_;
          } else {
            diff += std::abs(msg->position[i] - last_msg_.position[j]);
          }
        }

        ROS_DEBUG_STREAM_THROTTLE(0.1, "joint states diff: " << diff);

        if (diff > threshold_) {
          pub_.publish(*msg);
          last_msg_ = *msg;
        }
      }

      boost::mutex mutex_;
      ros::NodeHandle nh_, pnh_;
      ros::Publisher pub_;
      ros::Subscriber sub_;
      urdf::Model robot_model_;
      std::vector<std::string> blacklist_;
      sensor_msgs::JointState last_msg_;
      boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
      bool use_sim_time_;
      bool periodic_;
      double periodic_rate_;
      double threshold_;
      double rotation_factor_;
    };
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "joint_states_throttle");

  jsk_robot_startup::lifelog::JointStatesThrottle n;

  ros::spin();

  return 0;
}
