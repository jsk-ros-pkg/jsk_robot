// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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
 * pr2_base_trajectory_action_controller.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef PR2_BASE_TRAJECTORY_ACTION_CONTROLLER_H__
#define PR2_BASE_TRAJECTORY_ACTION_CONTROLLER_H__

#include <algorithm>
#include <string>
#include <stdexcept>
//#include <boost/range/algorithm.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <nav_msgs/Odometry.h>

#include <pr2_base_trajectory_action/math_spline.h>

namespace pr2_base_trajectory_action {
  typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> Server;
  typedef Server::GoalHandle GoalHandle;

  struct JointState {
    std::string name;
    double position, velocity;
    double max_velocity;
    double commanded_velocity;
    double commanded_effort;
  };

  struct BaseJointState {
    ros::Time real_latest_time;
    JointState x, y, yaw;
    const JointState &operator[] (size_t i) const {
      if (i == 0) return x;
      else if (i == 1) return y;
      else if (i == 2) return yaw;
      else throw std::out_of_range("range must be 0-2");
    }
    JointState &operator[] (size_t i) {
      if (i == 0) return x;
      else if (i == 1) return y;
      else if (i == 2) return yaw;
      else throw std::out_of_range("range must be 0-2");
    }
  };

  struct Trajectory {
    struct Segment
    {
      double start_time;
      double duration;
      Spline splines[3];
      boost::shared_ptr<GoalHandle> gh;
      Segment(){}
      Segment(const double &start_time,
              const double &duration,
              const boost::shared_ptr<GoalHandle> &gh,
              const Spline &spline_x,
              const Spline &spline_y,
              const Spline &spline_yaw) :
        start_time(start_time),
        duration(duration),
        gh(gh) {
        splines[0] = spline_x;
        splines[1] = spline_y;
        splines[2] = spline_yaw;
      }
      void sample(const double &time, BaseMotion &base) {
        for (int i = 0; i < 3; ++i)
          splines[i].sampleWithTimeBounds(duration, time, base[i]);
      }
      typedef boost::shared_ptr< ::pr2_base_trajectory_action::Trajectory::Segment > Ptr;
    };
    std::vector<Segment> segments;

    Trajectory() {}
    Trajectory(int size) : segments(size) {}

    bool nextSegment(const double &time, Segment &seg) {
      if (size() == 0) {
        ROS_ERROR("no segments in trajectory");
        return false;
      }
      int seg_idx = -1;
      while (seg_idx + 1 < size() &&
             segments[seg_idx+1].start_time < time)
        ++seg_idx;
      if (seg_idx == -1) {
        ROS_ERROR("No earlier segments. Segment starts at %.3lf (now: %.3lf)",
                  segments[0].start_time, time);
        return false;
      }
      seg = segments[seg_idx];
      return true;
    }
    bool motionAtTime(const double &time, BaseMotion &base) {
      Segment s;
      if (!nextSegment(time, s)) return false;
      double diff_time = time - s.start_time;
      s.sample(diff_time, base);
      return true;
    }
    bool goalAtTime(const double &time, boost::shared_ptr<GoalHandle> &gh) {
      Segment s;
      if (!nextSegment(time, s)) return false;
      gh = s.gh;
      return true;
    }
    double endTime() {
      Segment s = segments[size() - 1];
      return s.start_time + s.duration;
    }
    inline int size() const {
      return segments.size();
    }
    typedef boost::shared_ptr< ::pr2_base_trajectory_action::Trajectory > Ptr;
    typedef boost::shared_ptr< ::pr2_base_trajectory_action::Trajectory const > ConstPtr;
  };

  class Controller
  {

    boost::mutex mutex_;
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Timer update_timer_;
    boost::shared_ptr<Server> as_;
    boost::shared_ptr<GoalHandle> active_goal_;
    Trajectory::Ptr active_traj_;
    ros::Time real_latest_time_;
    BaseJointState joints_;
    double frequency_;
    double stopped_velocity_tolerance_;
    double goal_threshold_;
    double goal_time_constraint_;
    bool use_pid;
    control_msgs::FollowJointTrajectoryFeedback::Ptr feedback_msg_;
    geometry_msgs::Twist::Ptr cmd_vel_msg_;
  
    void setFeedback(const ros::Time &time, const BaseMotion &desired, const BaseMotion &error);
    void clearTrajectory();
    void setTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &msg,
                       boost::shared_ptr<GoalHandle> &gh);
  public:

    Controller();
    virtual ~Controller();
    void goalCallback(GoalHandle gh);
    void cancelCallback(GoalHandle gh);
    void odomCallback(const nav_msgs::Odometry::ConstPtr & msg);
    void update(const ros::TimerEvent& event);
  }; // class

  template<typename T>
  inline int findIndex(std::vector<T> vec, T val)
  {
    typename
      std::vector<T>::iterator it = std::find(vec.begin(), vec.end(), val);
    int d = it - vec.begin();
    return d == vec.size() ? -1 : d;
  }

  template <typename E, typename M>
  inline boost::shared_ptr<M> share_member(boost::shared_ptr<E> e, M &m)
  {
    actionlib::EnclosureDeleter<E> d(e);
    boost::shared_ptr<M> p(&m, d);
    return p;
  }
} // namespace


#endif // PR2_BASE_TRAJECTORY_ACTION_CONTROLLER_H__
