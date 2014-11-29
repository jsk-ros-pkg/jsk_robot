/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

/*!
  \author Stuart Glaser

  \class pr2_controller_interface::JointTrajectoryActionController

*/

#ifndef JOINT_TRAJECTORY_ACTION_CONTROLLER_H__
#define JOINT_TRAJECTORY_ACTION_CONTROLLER_H__

#include <vector>

#include <boost/scoped_ptr.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>
#include <ros/node_handle.h>

#include <actionlib/server/action_server.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

class BaseTrajectoryActionController
{
  typedef actionlib::ActionServer<pr2_controllers_msgs::JointTrajectoryAction> JTAS;
  typedef JTAS::GoalHandle GoalHandle;

  // dummy class
  class JointState {
  public:
    std::string name;
    double position_, velocity_;
    double max_abs_velocity_;
    double commanded_velocity_;
    double commanded_effort_;// same unit as velocity
  };

public:

  BaseTrajectoryActionController(const ros::NodeHandle& n);
  ~BaseTrajectoryActionController();

  bool init();

  void starting();
  void update();

private:
  bool use_pid;
  JointState x_joint_,y_joint_,z_joint_;
  ros::Time last_time_, robot_time_;
  std::vector<JointState*> joints_;
  double goal_time_constraint_;
  double goal_threshold_;
  double stopped_velocity_tolerance_;
  std::vector<double> goal_constraints_;
  std::vector<double> trajectory_constraints_;

  ros::NodeHandle node_;

  void commandCB(const trajectory_msgs::JointTrajectory::ConstPtr &msg);
  ros::Publisher pub_command_;

  void odomCB(const nav_msgs::Odometry::ConstPtr& msg);
  ros::Subscriber sub_odom_;

  boost::scoped_ptr<JTAS> action_server_;
  void goalCB(GoalHandle gh);
  void cancelCB(GoalHandle gh);
  ros::Timer goal_handle_timer_;

  boost::shared_ptr<GoalHandle> active_goal_;

  void commandTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &traj, boost::shared_ptr<GoalHandle> gh = boost::shared_ptr<GoalHandle>((GoalHandle*)NULL));

  // coef[0] + coef[1]*t + ... + coef[5]*t^5
  struct Spline
  {
    std::vector<double> coef;

    Spline() : coef(6, 0.0) {}
  };

  struct Segment
  {
    double start_time;
    double duration;
    std::vector<Spline> splines;

    boost::shared_ptr<GoalHandle> gh;
  };
  typedef std::vector<Segment> SpecifiedTrajectory;

  boost::shared_ptr<const SpecifiedTrajectory> current_trajectory_;

  // Holds the trajectory that we are currently following.  The mutex
  // guarding current_trajectory_ is locked from within realtime, so
  // it may only be locked for a bounded duration.
  //boost::shared_ptr<const SpecifiedTrajectory> current_trajectory_;
  //boost::recursive_mutex current_trajectory_lock_RT_;

  std::vector<double> q, qd, qdd;  // Preallocated in init
  std::vector<double> e, ed, edd;  // Preallocated in init

  // Samples, but handling time bounds.  When the time is past the end
  // of the spline duration, the position is the last valid position,
  // and the derivatives are all 0.
  static void sampleSplineWithTimeBounds(const std::vector<double>& coefficients, double duration, double time,
                                         double& position, double& velocity, double& acceleration);
};


#endif
