/*
 * Copyright (c) 2011, JSK lab.
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
 *     * Neither the name of JSK, university of Tokyo, nor the names of its
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

// Author: Manabu Saito

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_base_trajectory_action/pr2_base_trajectory_action.h>

#include <sstream>
#include "angles/angles.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "base_trajectory_action_node");
  ros::NodeHandle node;//("~");
  BaseTrajectoryActionController ac(node);

  ros::Rate r(40.0);
  while(ros::ok()) {
    ros::spinOnce();
    ac.update();
    r.sleep();
  }

  return 0;
}

static inline void generatePowers(int n, double x, double* powers)
{
  powers[0] = 1.0;
  for (int i=1; i<=n; i++)
  {
    powers[i] = powers[i-1]*x;
  }
}

static void getQuinticSplineCoefficients(double start_pos, double start_vel, double start_acc,
    double end_pos, double end_vel, double end_acc, double time, std::vector<double>& coefficients)
{
  coefficients.resize(6);

  if (time == 0.0)
  {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.5*end_acc;
    coefficients[3] = 0.0;
    coefficients[4] = 0.0;
    coefficients[5] = 0.0;
  }
  else
  {
    double T[6];
    generatePowers(5, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = 0.5*start_acc;
    coefficients[3] = (-20.0*start_pos + 20.0*end_pos - 3.0*start_acc*T[2] + end_acc*T[2] -
                       12.0*start_vel*T[1] - 8.0*end_vel*T[1]) / (2.0*T[3]);
    coefficients[4] = (30.0*start_pos - 30.0*end_pos + 3.0*start_acc*T[2] - 2.0*end_acc*T[2] +
                       16.0*start_vel*T[1] + 14.0*end_vel*T[1]) / (2.0*T[4]);
    coefficients[5] = (-12.0*start_pos + 12.0*end_pos - start_acc*T[2] + end_acc*T[2] -
                       6.0*start_vel*T[1] - 6.0*end_vel*T[1]) / (2.0*T[5]);
  }
}

/**
 * \brief Samples a quintic spline segment at a particular time
 */
static void sampleQuinticSpline(const std::vector<double>& coefficients, double time,
    double& position, double& velocity, double& acceleration)
{
  // create powers of time:
  double t[6];
  generatePowers(5, time, t);

  position = t[0]*coefficients[0] +
      t[1]*coefficients[1] +
      t[2]*coefficients[2] +
      t[3]*coefficients[3] +
      t[4]*coefficients[4] +
      t[5]*coefficients[5];

  velocity = t[0]*coefficients[1] +
      2.0*t[1]*coefficients[2] +
      3.0*t[2]*coefficients[3] +
      4.0*t[3]*coefficients[4] +
      5.0*t[4]*coefficients[5];

  acceleration = 2.0*t[0]*coefficients[2] +
      6.0*t[1]*coefficients[3] +
      12.0*t[2]*coefficients[4] +
      20.0*t[3]*coefficients[5];
}

static void getCubicSplineCoefficients(double start_pos, double start_vel,
    double end_pos, double end_vel, double time, std::vector<double>& coefficients)
{
  coefficients.resize(4);

  if (time == 0.0)
  {
    coefficients[0] = end_pos;
    coefficients[1] = end_vel;
    coefficients[2] = 0.0;
    coefficients[3] = 0.0;
  }
  else
  {
    double T[4];
    generatePowers(3, time, T);

    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = (-3.0*start_pos + 3.0*end_pos - 2.0*start_vel*T[1] - end_vel*T[1]) / T[2];
    coefficients[3] = (2.0*start_pos - 2.0*end_pos + start_vel*T[1] + end_vel*T[1]) / T[3];
  }
}


BaseTrajectoryActionController::BaseTrajectoryActionController(const ros::NodeHandle &nh)
  : node_(nh)
{
  joints_.push_back(&x_joint_);
  joints_.push_back(&y_joint_);
  joints_.push_back(&z_joint_);

  init();
}

BaseTrajectoryActionController::~BaseTrajectoryActionController()
{
  pub_command_.shutdown();
  sub_odom_.shutdown();
  action_server_.reset();
}

bool BaseTrajectoryActionController::init()
{
  using namespace XmlRpc;

  // virtual joint settings
  std::string ns = std::string("joint_trajectory_action");
  node_.param(ns+"/x_joint_name", x_joint_.name, std::string("base_link_x"));
  node_.param(ns+"/y_joint_name", y_joint_.name, std::string("base_link_y"));
  node_.param(ns+"/rotational_joint_name", z_joint_.name,
			  std::string("base_link_pan"));

  node_.param(ns+"/use_pid", use_pid, true);
  node_.param(ns+"/goal_threshold", goal_threshold_, 0.01);
  node_.param(ns+"/stopped_velocity_tolerance", stopped_velocity_tolerance_, 0.01);
  node_.param(ns+"/goal_time_constraint", goal_time_constraint_, 0.0);

  for (size_t i = 0; i < joints_.size(); ++i)
    {
      node_.param(ns + "/" + joints_[i]->name + "/max_velocity",
				  joints_[i]->max_abs_velocity_, 0.2);
      ROS_INFO("%s %f", ns.c_str(), joints_[i]->max_abs_velocity_);
    }

  // Creates a dummy trajectory
  boost::shared_ptr<SpecifiedTrajectory> traj_ptr(new SpecifiedTrajectory(1));
  SpecifiedTrajectory &traj = *traj_ptr;
  traj[0].start_time = ros::Time::now().toSec();
  traj[0].duration = 0.0;
  traj[0].splines.resize(joints_.size());
  for (size_t j = 0; j < joints_.size(); ++j)
    traj[0].splines[j].coef[0] = 0.0;
  current_trajectory_ = traj_ptr;

  pub_command_ = node_.advertise<geometry_msgs::Twist>("command", 10);
  sub_odom_ = node_.subscribe<nav_msgs::Odometry>("odom", 1, boost::bind(&BaseTrajectoryActionController::odomCB, this, _1));

  q.resize(joints_.size());
  qd.resize(joints_.size());
  qdd.resize(joints_.size());
  e.resize(joints_.size());
  ed.resize(joints_.size());
  edd.resize(joints_.size());

  action_server_.reset(new JTAS(node_, "joint_trajectory_action",
                                boost::bind(&BaseTrajectoryActionController::goalCB, this, _1),
                                boost::bind(&BaseTrajectoryActionController::cancelCB, this, _1)));

  ROS_DEBUG("base traj controller initialized;");

  return true;
}

void BaseTrajectoryActionController::starting()
{
  last_time_ = ros::Time::now();

  // Creates a "hold current position" trajectory.
  boost::shared_ptr<SpecifiedTrajectory> hold_ptr(new SpecifiedTrajectory(1));
  SpecifiedTrajectory &hold = *hold_ptr;
  hold[0].start_time = last_time_.toSec() - 0.001;
  hold[0].duration = 0.0;
  hold[0].splines.resize(joints_.size());
  for (size_t j = 0; j < joints_.size(); ++j)
    hold[0].splines[j].coef[0] = joints_[j]->position_;

  current_trajectory_ = hold_ptr;
}

void BaseTrajectoryActionController::update()
{
  ros::Time time = ros::Time::now();// + ros::Duration(0.05);
  ros::Duration dt = time - last_time_;
  last_time_ = time;

  if(active_goal_ == NULL) return;

  boost::shared_ptr<const SpecifiedTrajectory> traj_ptr;
  traj_ptr = current_trajectory_;
  if (!traj_ptr)
    ROS_FATAL("The current trajectory can never be null");

  // Only because this is what the code originally looked like.
  const SpecifiedTrajectory &traj = *traj_ptr;

  // ------ Finds the current segment

  // Determines which segment of the trajectory to use.  (Not particularly realtime friendly).
  int seg = -1;
  while (seg + 1 < (int)traj.size() &&
         traj[seg+1].start_time < time.toSec())
  {
    ++seg;
  }

  if (seg == -1)
  {
    if (traj.size() == 0)
      ROS_ERROR("No segments in the trajectory");
    else
      ROS_ERROR("No earlier segments.  First segment starts at %.3lf (now = %.3lf)", traj[0].start_time, time.toSec());
    return;
  }

  // ------ Trajectory Sampling

  for (size_t i = 0; i < q.size(); ++i)
  {
    sampleSplineWithTimeBounds(traj[seg].splines[i].coef, traj[seg].duration,
                               time.toSec() - traj[seg].start_time,
                               q[i], qd[i], qdd[i]);
  }

  // Defference between odometry and sample point
  std::vector<double> error(joints_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
    {
      // no odometry info
      if(robot_time_ == ros::Time(0)) continue;

      sampleSplineWithTimeBounds(traj[seg].splines[i].coef, traj[seg].duration,
				 robot_time_.toSec() - traj[seg].start_time,
				 e[i], ed[i], edd[i]);
      if(joints_[i] == &z_joint_) {
	error[i] = angles::shortest_angular_distance(joints_[i]->position_, e[i]);
      } else {
	error[i] = e[i] - joints_[i]->position_;
      }

      joints_[i]->commanded_effort_ = error[i] * 1.0; // P_gain:1.0
    }


  // ------ Determines if the goal has failed or succeeded

  if (traj[seg].gh && traj[seg].gh == active_goal_)
  {
    ros::Time end_time(traj[seg].start_time + traj[seg].duration);
    ROS_DEBUG("time: %f -> end: %f", time.toSec(), end_time.toSec());
    ROS_DEBUG("seg = %d, tarj.size() = %ld", seg, traj.size());
    if (time <= end_time)
    {
      // Verifies trajectory constraints
      for (size_t j = 0; j < joints_.size(); ++j)
      {
      }
    }
    else if (seg == (int)traj.size() - 1)
    {
      // Checks that we have ended inside the goal constraints
      bool inside_goal_constraints = true;
      for (size_t i = 0; i < joints_.size() && inside_goal_constraints; ++i)
      {
        // It's important to be stopped if that's desired.
        if (fabs(qd[i]) < 1e-6)
        {
          ROS_DEBUG("qd[%ld] = %f", i, qd[i]);
          if (fabs(joints_[i]->velocity_) > stopped_velocity_tolerance_) {
            ROS_WARN("fabs(joints_[%ld]->velocity_) = %f > %f (stopped_velocity_tolerance_)",
                      i, joints_[i]->velocity_, stopped_velocity_tolerance_);
            inside_goal_constraints = false;
          }
        }
      }

      for (size_t i = 0; i < joints_.size() && inside_goal_constraints; ++i)
      {
        if(fabs(error[i]) > goal_threshold_) {
          ROS_WARN("fabs(error[%ld]) = %f > %f (goal_threshold)",
                   i, fabs(error[i]), goal_threshold_);
          inside_goal_constraints = false;
        }
      }

      if (inside_goal_constraints)
      {
	traj[seg].gh->setSucceeded();
	active_goal_ = boost::shared_ptr<GoalHandle>((GoalHandle*)NULL);
	ROS_DEBUG("base_traj success");
        //ROS_ERROR("Success!  (%s)", traj[seg].gh->gh_.getGoalID().id.c_str());
      }
      else if (time < end_time + ros::Duration(goal_time_constraint_))
      {
        // Still have some time left to make it.
      }
      else
      {
        //ROS_WARN("Aborting because we wound up outside the goal constraints");
        traj[seg].gh->setAborted();
	active_goal_ = boost::shared_ptr<GoalHandle>((GoalHandle*)NULL);
	ROS_WARN("base_traj aborted");
      }
    }

    // check if new command violates the max velocity
    for (size_t j = 0; j < joints_.size(); ++j) {
      joints_[j]->commanded_velocity_ = qd[j] + (use_pid ? joints_[j]->commanded_effort_ : 0);
      if(joints_[j]->max_abs_velocity_ < fabs(joints_[j]->commanded_velocity_))
      {
	joints_[j]->commanded_velocity_ *=
	  joints_[j]->max_abs_velocity_/fabs(joints_[j]->commanded_velocity_);
	ROS_WARN("joint(%s) violates its velocity limit.", joints_[j]->name.c_str());
      }
   }

    // publish controll velocity
    geometry_msgs::Twist vel;
    double vx,vy,theta; // in odometry space
    vx = joints_[0]->commanded_velocity_;
    vy = joints_[1]->commanded_velocity_;
    theta = joints_[2]->position_;
    vel.linear.x = vx * cos(theta) + vy * sin(theta);
    vel.linear.y = vy * cos(theta) - vx * sin(theta);
    vel.angular.z = joints_[2]->commanded_velocity_;
    pub_command_.publish(vel);

    ROS_DEBUG("pos = %f %f %f", joints_[0]->position_,joints_[1]->position_,joints_[2]->position_);
    ROS_DEBUG("vel = %f %f %f", vel.linear.x, vel.linear.y, vel.angular.z);
  }

}

void BaseTrajectoryActionController::commandTrajectory(const trajectory_msgs::JointTrajectory::ConstPtr &msg, boost::shared_ptr<GoalHandle> gh)
{
  ros::Time time = last_time_ + ros::Duration(0.01);
  ROS_DEBUG("Figuring out new trajectory at %.3lf, with data from %.3lf",
            time.toSec(), msg->header.stamp.toSec());

  boost::shared_ptr<SpecifiedTrajectory> new_traj_ptr(new SpecifiedTrajectory);
  SpecifiedTrajectory &new_traj = *new_traj_ptr;

  // ------ If requested, performs a stop

  if (msg->points.empty())
  {
    starting();
    return;
  }

  // ------ Correlates the joints we're commanding to the joints in the message

  std::vector<int> lookup(joints_.size(), -1);  // Maps from an index in joints_ to an index in the msg
  for (size_t j = 0; j < joints_.size(); ++j)
  {
    for (size_t k = 0; k < msg->joint_names.size(); ++k)
    {
      if (msg->joint_names[k] == joints_[j]->name)
      {
        lookup[j] = k;
        break;
      }
    }

    if (lookup[j] == -1)
    {
      ROS_ERROR("Unable to locate joint %s in the commanded trajectory.", joints_[j]->name.c_str());
      return;
    }
  }

  // ------ Grabs the trajectory that we're currently following.

  boost::shared_ptr<const SpecifiedTrajectory> prev_traj_ptr;
  prev_traj_ptr =  current_trajectory_;
  if (!prev_traj_ptr)
  {
    ROS_FATAL("The current trajectory can never be null");
    return;
  }
  const SpecifiedTrajectory &prev_traj = *prev_traj_ptr;

  // ------ Copies over the segments from the previous trajectory that are still useful.

  // Useful segments are still relevant after the current time.
  int first_useful = -1;
  while (first_useful + 1 < (int)prev_traj.size() &&
         prev_traj[first_useful + 1].start_time <= time.toSec())
  {
    ++first_useful;
  }

  // Useful segments are not going to be completely overwritten by the message's splines.
  int last_useful = -1;
  double msg_start_time;
  if (msg->header.stamp == ros::Time(0.0))
    msg_start_time = time.toSec();
  else
    msg_start_time = msg->header.stamp.toSec();
  /*
  if (msg->points.size() > 0)
    msg_start_time += msg->points[0].time_from_start.toSec();
  */

  while (last_useful + 1 < (int)prev_traj.size() &&
         prev_traj[last_useful + 1].start_time < msg_start_time)
  {
    ++last_useful;
  }

  if (last_useful < first_useful)
    first_useful = last_useful;

  // Copies over the old segments that were determined to be useful.
  for (int i = std::max(first_useful,0); i <= last_useful; ++i)
  {
    new_traj.push_back(prev_traj[i]);
  }

  // We always save the last segment so that we know where to stop if
  // there are no new segments.
  if (new_traj.size() == 0)
    new_traj.push_back(prev_traj[prev_traj.size() - 1]);

  // ------ Determines when and where the new segments start

  // Finds the end conditions of the final segment
  Segment &last = new_traj[new_traj.size() - 1];
  std::vector<double> prev_positions(joints_.size());
  std::vector<double> prev_velocities(joints_.size());
  std::vector<double> prev_accelerations(joints_.size());

  double t = (msg->header.stamp == ros::Time(0.0) ? time.toSec() : msg->header.stamp.toSec())
    - last.start_time;
  ROS_DEBUG("Initial conditions at %.3f for new set of splines:", t);
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    sampleSplineWithTimeBounds(last.splines[i].coef, last.duration,
                               t,
                               prev_positions[i], prev_velocities[i], prev_accelerations[i]);
    ROS_DEBUG("    %.2lf, %.2lf, %.2lf  (%s)", prev_positions[i], prev_velocities[i],
              prev_accelerations[i], joints_[i]->name.c_str());
  }

  // ------ Tacks on the new segments

  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> accelerations;

  std::vector<double> durations(msg->points.size());
  if (msg->points.size() > 0)
    durations[0] = msg->points[0].time_from_start.toSec();
  for (size_t i = 1; i < msg->points.size(); ++i)
    durations[i] = (msg->points[i].time_from_start - msg->points[i-1].time_from_start).toSec();

  // Checks if we should wrap
  std::vector<double> wrap(joints_.size(), 0.0);
  if (msg->points[0].positions.empty()) {
    ROS_ERROR("First point of trajectory has no positions");
    return;
  }
  ROS_DEBUG("wrap:");
  for (size_t j = 0; j < joints_.size(); ++j)
  {
    if (joints_[j] == &z_joint_) // z_joint_ angle is -pi <-> pi [rad]
    {
      double dist = angles::shortest_angular_distance(prev_positions[j], msg->points[0].positions[lookup[j]]);
      wrap[j] = (prev_positions[j] + dist) - msg->points[0].positions[lookup[j]];
      ROS_DEBUG("    %.2lf  - %s bc dist(%.2lf, %.2lf) = %.2lf", wrap[j], joints_[j]->name.c_str(),
                prev_positions[j], msg->points[0].positions[lookup[j]], dist);
    }
  }

  for (size_t i = 0; i < msg->points.size(); ++i)
  {
    Segment seg;

    if (msg->header.stamp == ros::Time(0.0))
      seg.start_time = (time + msg->points[i].time_from_start).toSec() - durations[i];
    else
      seg.start_time = (msg->header.stamp + msg->points[i].time_from_start).toSec() - durations[i];
    seg.duration = durations[i];
    seg.gh = gh;
    seg.splines.resize(joints_.size());

    // Checks that the incoming segment has the right number of elements.

    if (msg->points[i].accelerations.size() != 0 && msg->points[i].accelerations.size() != joints_.size())
    {
      ROS_ERROR("Command point %d has %d elements for the accelerations", (int)i, (int)msg->points[i].accelerations.size());
      return;
    }
    if (msg->points[i].velocities.size() != 0 && msg->points[i].velocities.size() != joints_.size())
    {
      ROS_ERROR("Command point %d has %d elements for the velocities", (int)i, (int)msg->points[i].velocities.size());
      return;
    }
    if (msg->points[i].positions.size() != joints_.size())
    {
      ROS_ERROR("Command point %d has %d elements for the positions", (int)i, (int)msg->points[i].positions.size());
      return;
    }

    // Re-orders the joints in the command to match the internal joint order.

    accelerations.resize(msg->points[i].accelerations.size());
    velocities.resize(msg->points[i].velocities.size());
    positions.resize(msg->points[i].positions.size());
    for (size_t j = 0; j < joints_.size(); ++j)
    {
      if (!accelerations.empty()) accelerations[j] = msg->points[i].accelerations[lookup[j]];
      if (!velocities.empty()) velocities[j] = msg->points[i].velocities[lookup[j]];
      if (!positions.empty()) positions[j] = msg->points[i].positions[lookup[j]] + wrap[j];
    }

    // Converts the boundary conditions to splines.

    for (size_t j = 0; j < joints_.size(); ++j)
    {
      if (prev_accelerations.size() > 0 && accelerations.size() > 0)
      {
        getQuinticSplineCoefficients(
          prev_positions[j], prev_velocities[j], prev_accelerations[j],
          positions[j], velocities[j], accelerations[j],
          durations[i],
          seg.splines[j].coef);
      }
      else if (prev_velocities.size() > 0 && velocities.size() > 0)
      {
        getCubicSplineCoefficients(
          prev_positions[j], prev_velocities[j],
          positions[j], velocities[j],
          durations[i],
          seg.splines[j].coef);
        seg.splines[j].coef.resize(6, 0.0);
      }
      else
      {
        seg.splines[j].coef[0] = prev_positions[j];
        if (durations[i] == 0.0)
          seg.splines[j].coef[1] = 0.0;
        else
          seg.splines[j].coef[1] = (positions[j] - prev_positions[j]) / durations[i];
        seg.splines[j].coef[2] = 0.0;
        seg.splines[j].coef[3] = 0.0;
        seg.splines[j].coef[4] = 0.0;
        seg.splines[j].coef[5] = 0.0;
      }
    }

    // Pushes the splines onto the end of the new trajectory.

    new_traj.push_back(seg);

    // Computes the starting conditions for the next segment

    prev_positions = positions;
    prev_velocities = velocities;
    prev_accelerations = accelerations;
  }

  //ROS_ERROR("Last segment goal id: %s", new_traj[new_traj.size()-1].gh->gh_.getGoalID().id.c_str());

  // ------ Commits the new trajectory

  if (!new_traj_ptr)
  {
    ROS_ERROR("The new trajectory was null!");
    return;
  }

  current_trajectory_ = new_traj_ptr;
  ROS_DEBUG("The new trajectory has %d segments", (int)new_traj.size());
#if 0
  for (size_t i = 0; i < std::min((size_t)20,new_traj.size()); ++i)
  {
    ROS_DEBUG("Segment %2ld: %.3lf for %.3lf", i, new_traj[i].start_time, new_traj[i].duration);
    for (size_t j = 0; j < new_traj[i].splines.size(); ++j)
    {
      ROS_DEBUG("    %.2lf  %.2lf  %.2lf  %.2lf , %.2lf  %.2lf(%s)",
                new_traj[i].splines[j].coef[0],
                new_traj[i].splines[j].coef[1],
                new_traj[i].splines[j].coef[2],
                new_traj[i].splines[j].coef[3],
                new_traj[i].splines[j].coef[4],
                new_traj[i].splines[j].coef[5],
                joints_[j]->name.c_str());
    }
  }
#endif
}

void BaseTrajectoryActionController::odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  robot_time_ = msg->header.stamp;
  x_joint_.position_ = msg->pose.pose.position.x;
  y_joint_.position_ = msg->pose.pose.position.y;
  z_joint_.position_ = atan2(msg->pose.pose.orientation.z,
			      msg->pose.pose.orientation.w) * 2;
  //x_joint_.velocity_ = msg->twist.twist.linear.x;
  //y_joint_.velocity_ = msg->twist.twist.linear.y;
  //z_joint_.velocity_ = msg->twist.twist.angular.z;
}

void BaseTrajectoryActionController::sampleSplineWithTimeBounds(
  const std::vector<double>& coefficients, double duration, double time,
  double& position, double& velocity, double& acceleration)
{
  if (time < 0)
  {
    double _;
    sampleQuinticSpline(coefficients, 0.0, position, _, _);
    velocity = 0;
    acceleration = 0;
  }
  else if (time > duration)
  {
    double _;
    sampleQuinticSpline(coefficients, duration, position, _, _);
    velocity = 0;
    acceleration = 0;
  }
  else
  {
    sampleQuinticSpline(coefficients, time,
                        position, velocity, acceleration);
  }
}

static bool setsEqual(const std::vector<std::string> &a, const std::vector<std::string> &b)
{
  if (a.size() != b.size())
    return false;

  for (size_t i = 0; i < a.size(); ++i)
  {
    if (count(b.begin(), b.end(), a[i]) != 1)
      return false;
  }
  for (size_t i = 0; i < b.size(); ++i)
  {
    if (count(a.begin(), a.end(), b[i]) != 1)
      return false;
  }

  return true;
}

template <class Enclosure, class Member>
static boost::shared_ptr<Member> share_member(boost::shared_ptr<Enclosure> enclosure, Member &member)
{
  actionlib::EnclosureDeleter<Enclosure> d(enclosure);
  boost::shared_ptr<Member> p(&member, d);
  return p;
}


void BaseTrajectoryActionController::goalCB(GoalHandle gh)
{
  ROS_DEBUG("goalCB");

  std::vector<std::string> joint_names(joints_.size());
  for (size_t j = 0; j < joints_.size(); ++j)
    joint_names[j] = joints_[j]->name;

  // Ensures that the joints in the goal match the joints we are commanding.
  if (!setsEqual(joint_names, gh.getGoal()->trajectory.joint_names))
  {
    ROS_ERROR("Joints on incoming goal don't match our joints");
    gh.setRejected();
    return;
  }

  // Cancels the currently active goal.
  if (active_goal_)
  {
    // Marks the current goal as canceled.
    active_goal_->setCanceled();
  }

  starting(); // reset trajectory

  gh.setAccepted();

  // Sends the trajectory along to the controller
  active_goal_ = boost::shared_ptr<GoalHandle>(new GoalHandle(gh));
  commandTrajectory(share_member(gh.getGoal(),gh.getGoal()->trajectory), active_goal_);
}

void BaseTrajectoryActionController::cancelCB(GoalHandle gh)
{
  if (active_goal_ && *active_goal_ == gh)
  {
    trajectory_msgs::JointTrajectory::Ptr empty(new trajectory_msgs::JointTrajectory);
    empty->joint_names.resize(joints_.size());
    for (size_t j = 0; j < joints_.size(); ++j)
      empty->joint_names[j] = joints_[j]->name;
    commandTrajectory(empty);

    // Marks the current goal as canceled.
    active_goal_->setCanceled();
    active_goal_ = boost::shared_ptr<GoalHandle>((GoalHandle*)NULL);
  }
}
