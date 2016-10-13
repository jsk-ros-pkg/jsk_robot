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
 * pr2_base_trajectory_action_controller.cpp
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <set>
#include <angles/angles.h>
#include <pr2_base_trajectory_action/pr2_base_trajectory_action_controller.h>
#include <geometry_msgs/Twist.h>

namespace pr2_base_trajectory_action {

  Controller::Controller()
  : nh_(""), pnh_("~")
  {
    pnh_.param("update_frequency", frequency_, 40.0);
    pnh_.param("stopped_velocity_tolerance", stopped_velocity_tolerance_, 0.01);
    pnh_.param("goal_threshold", goal_threshold_, 0.01);
    pnh_.param("goal_time_constraint", goal_time_constraint_, 0.0);

    pnh_.param<std::string>("linear_x_joint_name", joints_.x.name, "base_link_x");
    pnh_.param<std::string>("linear_y_joint_name", joints_.y.name, "base_link_y");
    pnh_.param<std::string>("rotational_z_joint_name", joints_.yaw.name, "base_link_pan");
    pnh_.param(joints_.x.name + "/max_velocity", joints_.x.max_velocity, 0.2);
    pnh_.param(joints_.y.name + "/max_velocity", joints_.y.max_velocity, 0.2);
    pnh_.param(joints_.yaw.name + "/max_velocity", joints_.yaw.max_velocity, 0.2);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("command", 10);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("odom", 1,
                                                  boost::bind(&Controller::odomCallback, this, _1));

    update_timer_ = nh_.createTimer(ros::Duration(1.0 / frequency_),
                                    boost::bind(&Controller::update, this, _1));

    as_.reset(new Server(nh_,
                         "follow_joint_trajectory",
                         boost::bind(&Controller::goalCallback, this, _1),
                         boost::bind(&Controller::cancelCallback, this, _1),
                         /* auto_start = */ false));
    as_->start();
  }

  Controller::~Controller()
  {}

  void Controller::goalCallback(GoalHandle gh)
  {
    boost::mutex::scoped_lock lock(mutex_);

    ROS_DEBUG("new goal arrived");

    // check joint names
    std::set<std::string> names;
    for (int i = 0; i < 3; ++i)
      names.insert(joints_[i].name);
    std::set<std::string> goal_joint_set(gh.getGoal()->trajectory.joint_names.begin(),
                                         gh.getGoal()->trajectory.joint_names.end());
    if (names != goal_joint_set) {
      ROS_ERROR("Joints on incoming goal don't match our joints");
      gh.setRejected();
      return;
    }

    // send cancel to current active goal
    if (active_goal_)
    {
      ROS_DEBUG("canceled active goal");
      active_goal_->setCanceled();
    }

    // validate points
    if (!gh.getGoal()->trajectory.points.empty())
    {
      int pos_size = gh.getGoal()->trajectory.points[0].positions.size();
      if (pos_size != gh.getGoal()->trajectory.points[0].velocities.size())
      {
        ROS_ERROR("size of points and velocities must be the same");
        gh.setRejected();
        return;
      }
      else if (!gh.getGoal()->trajectory.points[0].accelerations.empty() &&
               pos_size != gh.getGoal()->trajectory.points[0].accelerations.size())
      {
        ROS_ERROR("size of acclerations must be the same as points or 0");
        gh.setRejected();
        return;
      }
    }

    gh.setAccepted();
    active_goal_.reset(new GoalHandle(gh));

    ROS_DEBUG("goal accepted");

    if (gh.getGoal()->trajectory.points.empty())
    { // stop
      clearTrajectory();
      return;
    }

    // set new trajectory
    setTrajectory(share_member(gh.getGoal(), gh.getGoal()->trajectory), active_goal_);
  }

  void Controller::cancelCallback(GoalHandle gh)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (active_goal_ && *active_goal_ == gh)
    {
      trajectory_msgs::JointTrajectory::Ptr empty(new trajectory_msgs::JointTrajectory);
      boost::shared_ptr<GoalHandle> empty_gh = boost::shared_ptr<GoalHandle>((GoalHandle*)NULL);
      empty->joint_names.resize(3);
      for (int i = 0; i < 3; ++i)
        empty->joint_names[i] = joints_[i].name;
      setTrajectory(empty, empty_gh);
      active_goal_->setCanceled();
      active_goal_ = boost::shared_ptr<GoalHandle>((GoalHandle*)NULL);
    }
  }

  void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr & msg)
  {
    joints_.real_latest_time = msg->header.stamp;
    joints_.x.position = msg->pose.pose.position.x;
    joints_.y.position = msg->pose.pose.position.y;
    joints_.yaw.position = 2.0 * atan2(msg->pose.pose.orientation.z,
                                       msg->pose.pose.orientation.w);

    // for feedback msg
    joints_.x.velocity = msg->twist.twist.linear.x;
    joints_.y.velocity = msg->twist.twist.linear.y;
    joints_.yaw.velocity = msg->twist.twist.angular.z;
  }

  void Controller::update(const ros::TimerEvent& e)
  {
    if (!active_goal_) return;
    if (active_traj_->size() <= 0)
    {
      ROS_ERROR("No segments in active trajectory");
      return;
    }

    ros::Time now = e.current_real, last = e.last_real;
    ros::Duration dt = now - last;

    BaseMotion next_motion;
    active_traj_->motionAtTime(now.toSec(), next_motion);
    // calc error
    BaseMotion desired_motion, error_motion;
    active_traj_->motionAtTime(joints_.real_latest_time.toSec(), desired_motion);
    error_motion.x.position = desired_motion.x.position - joints_.x.position;
    error_motion.y.position = desired_motion.y.position - joints_.y.position;
    error_motion.yaw.position = angles::shortest_angular_distance(joints_.yaw.position,
                                                                  desired_motion.yaw.position);
    for(int i = 0; i < 3; ++i)
      joints_[i].commanded_effort = error_motion[i].position * 1.0; // P gain: 1.0

    // publish feedback
    setFeedback(now, desired_motion, error_motion);
    ROS_DEBUG_STREAM("desired: " << desired_motion);
    ROS_DEBUG_STREAM("actual: x: " << joints_.x.position <<
                     ", y: " << joints_.y.position <<
                     ", th: " << joints_.yaw.position);
    ROS_DEBUG_STREAM("error: " << error_motion);

    // check if goal reached
    boost::shared_ptr<GoalHandle> gh;
    bool has_goal = active_traj_->goalAtTime(now.toSec(), gh);
    if (has_goal && gh == active_goal_)
    {
      ros::Time end_time(active_traj_->endTime());
      ROS_DEBUG_STREAM("time: " << now.toSec() << " -> end: " << end_time.toSec());

      if(now > end_time) {
        ROS_DEBUG_STREAM("current time exceeded goal end time");
        bool inside_goal_constraints = true;
        for (size_t i = 0; i < 3 && inside_goal_constraints; ++i)
        {
          if (fabs(next_motion[i].velocity) < 1e-6)
          {
            if (fabs(joints_[i].velocity) > stopped_velocity_tolerance_)
            {
              ROS_WARN_STREAM("velocity of joint " << joints_[i].name << ": " << joints_[i].velocity << " > stopped_velocity_tolerance: " << stopped_velocity_tolerance_);
              inside_goal_constraints = false;
            } else if (fabs(error_motion[i].position) > goal_threshold_) {
              ROS_WARN_STREAM("error of joint " << joints_[i].name << ": " << error_motion[i].position << " > goal_threshold: " << goal_threshold_);
              inside_goal_constraints = false;
            }
          }
        }
        if (inside_goal_constraints)
        {
          active_goal_->setSucceeded();
          active_goal_ = boost::shared_ptr<GoalHandle>((GoalHandle*)NULL);
          ROS_DEBUG_STREAM("set succeeded");
        } else if (now >= end_time + ros::Duration(goal_time_constraint_)) {
          active_goal_->setAborted();
          active_goal_ = boost::shared_ptr<GoalHandle>((GoalHandle*)NULL);
          ROS_DEBUG_STREAM("set aborted");
        }
      }

      // check if new commands violates the max velocity
      for (size_t i = 0; i < 3; ++i)
      {
        joints_[i].commanded_velocity = desired_motion[i].velocity + (use_pid ? joints_[i].commanded_effort : 0);
        if (joints_[i].max_velocity < fabs(joints_[i].commanded_velocity))
        {
          joints_[i].commanded_velocity *= joints_[i].max_velocity / fabs(joints_[i].commanded_velocity);
          ROS_WARN_STREAM("joint " << joints_[i].name << " violates its velocity limit");
        }
      }


      // publish command velocity
      cmd_vel_msg_.reset(new geometry_msgs::Twist);
      double vx = joints_.x.commanded_velocity;
      double vy = joints_.y.commanded_velocity;
      double th = joints_.yaw.position;
      ROS_DEBUG_STREAM("vx: " << vx << ", vy: " << vy << ", th: " << th);
      cmd_vel_msg_->linear.x = vx * cos(th) + vy * sin(th);
      cmd_vel_msg_->linear.y = vy * cos(th) - vx * sin(th);
      cmd_vel_msg_->angular.z = joints_.yaw.commanded_velocity;
      ROS_DEBUG_STREAM("cmd_vel x: " << cmd_vel_msg_->linear.x <<
                      " ,y: " << cmd_vel_msg_->linear.y <<
                      " ,th: " << cmd_vel_msg_->angular.z);
      cmd_vel_pub_.publish(*cmd_vel_msg_);
    }
  } // function update

  void Controller::setFeedback(const ros::Time &stamp, const BaseMotion &desired, const BaseMotion &error)
  {
    if (!active_goal_) return;
    control_msgs::FollowJointTrajectoryFeedback msg;
    msg.header.stamp = stamp;
    for (int i = 0; i < 3; ++i) {
      msg.joint_names.push_back(joints_[i].name);
      msg.desired.positions.push_back(desired[i].position);
      msg.desired.velocities.push_back(desired[i].velocity);
      msg.actual.positions.push_back(joints_[i].position);
      msg.actual.velocities.push_back(joints_[i].velocity);
      msg.error.positions.push_back(error[i].position);
      msg.error.velocities.push_back(error[i].velocity);
    }
    active_goal_->publishFeedback(msg);
  }

  void Controller::setTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &msg,
                                 boost::shared_ptr<GoalHandle> &gh)
  {
    ROS_DEBUG("set trajectory");
    if (!active_traj_) clearTrajectory();
    ros::Time now = ros::Time::now();
    double msg_start_time = 0.0;
    if (msg->header.stamp == ros::Time(0.0))
      msg_start_time = now.toSec();
    else msg_start_time = msg->header.stamp.toSec();
    Trajectory::ConstPtr prev_traj = active_traj_;
    Trajectory::Ptr new_traj(new Trajectory(1));
    new_traj->segments[0].start_time = ros::Time::now().toSec() - 0.001;
    new_traj->segments[0].duration = 0.0;
    for (int i = 0; i < 3; ++i)
      new_traj->segments[0].splines[i].coefficients[0] = joints_[i].position;
    ROS_DEBUG("set prev trajectory");

    // first, append previous trajectory to new traecjtory
    int from_seg = -1, to_seg = -1;
    while (from_seg + 1 < prev_traj->size() &&
           prev_traj->segments[from_seg + 1].start_time <= now.toSec())
      ++from_seg;
    while (to_seg + 1 < prev_traj->size() &&
           prev_traj->segments[to_seg + 1].start_time < msg_start_time)
      ++to_seg;
    if (to_seg < from_seg) from_seg = to_seg;

    ROS_DEBUG_STREAM("use prev traj from " << from_seg << ", to " << to_seg);

    for (int i = std::max(from_seg, 0); i <= to_seg; ++i) {
      new_traj->segments.push_back(prev_traj->segments[i]);
    }
    if (new_traj->size() == 0)
      new_traj->segments.push_back(prev_traj->segments[prev_traj->size() - 1]);

    // look up table
    int joint_idx_x = findIndex(msg->joint_names, joints_.x.name);
    int joint_idx_y = findIndex(msg->joint_names, joints_.y.name);
    int joint_idx_yaw = findIndex(msg->joint_names, joints_.yaw.name);

    ROS_DEBUG_STREAM("look up x: " << joint_idx_x << ", y: " << joint_idx_y << ", yaw: " << joint_idx_yaw);

    // calc rotational align from previous trajectory
    Trajectory::Segment last_seg = new_traj->segments[new_traj->size() - 1];
    BaseMotion prev_motion;
    last_seg.sample(msg_start_time, prev_motion);
    double dist = angles::shortest_angular_distance(prev_motion.yaw.position,
                                                    msg->points[0].positions[joint_idx_yaw]);
    double yaw_align = prev_motion.yaw.position + dist - msg->points[0].positions[joint_idx_yaw];

    ROS_DEBUG_STREAM("yaw_align: " << yaw_align);

    // calc spline from trajectory
    for (size_t i = 0; i < msg->points.size(); ++i)
    {
      Trajectory::Segment s;
      s.duration = msg->points[i].time_from_start.toSec();
      if (i != 0) s.duration -= msg->points[i-1].time_from_start.toSec();
      s.start_time = msg_start_time + msg->points[i].time_from_start.toSec() - s.duration;
      s.gh = gh;

      ROS_DEBUG_STREAM("s.start_time: " << s.start_time << ", s.duration: " << s.duration);

      if (prev_motion.hasAcceleration() && msg->points[i].accelerations.size() > 0) {
        Motion mx(msg->points[i].positions[joint_idx_x],
                  msg->points[i].velocities[joint_idx_x],
                  msg->points[i].accelerations[joint_idx_x]);
        QuinticSpline spx(prev_motion.x, mx, s.duration);
        s.splines[0] = spx;

        Motion my(msg->points[i].positions[joint_idx_y],
                  msg->points[i].velocities[joint_idx_y],
                  msg->points[i].accelerations[joint_idx_y]);
        QuinticSpline spy(prev_motion.y, my, s.duration);
        s.splines[1] = spy;

        Motion myaw(msg->points[i].positions[joint_idx_yaw] + yaw_align,
                    msg->points[i].velocities[joint_idx_yaw],
                    msg->points[i].accelerations[joint_idx_yaw]);
        QuinticSpline spyaw(prev_motion.yaw, myaw, s.duration);
        s.splines[2] = spyaw;
        new_traj->segments.push_back(s);
        prev_motion.x = mx;
        prev_motion.y = my;
        prev_motion.yaw = myaw;
      } else if (prev_motion.hasVelocity() && msg->points[i].velocities.size() > 0) {
        ROS_DEBUG_THROTTLE(1.0, "velocity only");
        Motion mx(msg->points[i].positions[joint_idx_x],
                  msg->points[i].velocities[joint_idx_x]);
        CubicSpline spx(prev_motion.x, mx, s.duration);
        s.splines[0] = spx;

        Motion my(msg->points[i].positions[joint_idx_y],
                  msg->points[i].velocities[joint_idx_y]);
        CubicSpline spy(prev_motion.y, my, s.duration);
        s.splines[1] = spy;

        Motion myaw(msg->points[i].positions[joint_idx_yaw] + yaw_align,
                    msg->points[i].velocities[joint_idx_yaw]);
        CubicSpline spyaw(prev_motion.yaw, myaw, s.duration);
        s.splines[2] = spyaw;
        new_traj->segments.push_back(s);
        prev_motion.x = mx;
        prev_motion.y = my;
        prev_motion.yaw = myaw;
      } else {
        ROS_DEBUG_THROTTLE(1.0, "position only");
        Motion mx(msg->points[i].positions[joint_idx_x]);
        Line spx(prev_motion.x, mx, s.duration);
        s.splines[0] = spx;

        Motion my(msg->points[i].positions[joint_idx_y]);
        Line spy(prev_motion.y, my, s.duration);
        s.splines[1] = spy;

        Motion myaw(msg->points[i].positions[joint_idx_yaw] + yaw_align);
        Line spyaw(prev_motion.yaw, myaw, s.duration);
        s.splines[2] = spyaw;
        new_traj->segments.push_back(s);
        prev_motion.x = mx;
        prev_motion.y = my;
        prev_motion.yaw = myaw;
      }

      // debugging
      ROS_DEBUG_STREAM("start time: " << s.start_time << ", duration: " << s.duration);
      for (int j = 0; j < 3; ++j) {
        ROS_DEBUG_STREAM("  spline " << j << ":");
        for (int k = 0; k < 6; ++k)
          ROS_DEBUG_STREAM("    " << k << ": " << s.splines[j].coefficients[k]);
      }
    }

    // commit new active trajectory
    active_traj_ = new_traj;
    ROS_DEBUG_STREAM("new active trajectory is set");
  }

  void Controller::clearTrajectory()
  {
    active_traj_.reset(new Trajectory(1));
    active_traj_->segments[0].start_time = ros::Time::now().toSec() - 0.001;
    active_traj_->segments[0].duration = 0.0;
    for (int i = 0; i < 3; ++i)
      active_traj_->segments[0].splines[i].coefficients[0] = joints_[i].position;
    ROS_DEBUG("clear trajectory");
  }
} // namespace
