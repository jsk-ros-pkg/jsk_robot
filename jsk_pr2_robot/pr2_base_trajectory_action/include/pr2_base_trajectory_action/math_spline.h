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
 * math_spline.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef MATH_SPLINE_H__
#define MATH_SPLINE_H__

#include <iostream>
#include <vector>
#include <cfloat>
#include <stdexcept>
#include <boost/shared_ptr.hpp>

namespace pr2_base_trajectory_action {

  static inline void pows(int n, double base, double* ret)
  {
    ret[0] = 1.0;
    for(int i = 1; i <= n; ++i)
    {
      ret[i] = ret[i-1] * base;
    }
  }

  struct Motion
  {
    double position, velocity, acceleration;
    Motion() : position(0.0), velocity(DBL_MAX), acceleration(DBL_MAX) {}
    Motion(double position) : position(position), velocity(DBL_MAX), acceleration(DBL_MAX) {}
    Motion(double position, double velocity)
      : position(position), velocity(velocity), acceleration(DBL_MAX) {}
    Motion(double position, double velocity, double acceleration)
      : position(position), velocity(velocity), acceleration(acceleration) {}
    friend std::ostream &operator << (std::ostream &os, const Motion &m)
    {
      return os << "<pos: " << m.position << ", vel: " << m.velocity << ", acc: " << m.acceleration << ">";
    }
  };
  struct BaseMotion
  {
    Motion x, y, yaw;
    const Motion &operator[] (size_t i) const {
      switch (i) {
      case 0: return x;
      case 1: return y;
      case 2: return yaw;
      default: throw std::out_of_range("range must be 0-2");
      }
    }
    Motion &operator[] (size_t i) {
      switch (i) {
      case 0: return x;
      case 1: return y;
      case 2: return yaw;
      default: throw std::out_of_range("range must be 0-2");
      }
    }
    friend std::ostream &operator << (std::ostream &os, const BaseMotion &m) {
      return os << "<x: " << m.x.position << " y: " << m.y.position << " yaw: " << m.yaw.position << ">";
    }
    bool hasVelocity() const {
      return x.velocity != DBL_MAX && y.velocity != DBL_MAX && yaw.velocity != DBL_MAX;
    }
    bool hasAcceleration() const {
      return x.acceleration != DBL_MAX && y.acceleration != DBL_MAX && yaw.acceleration != DBL_MAX;
    }
  };

  struct Spline
  {
    std::vector<double> coefficients;
    Spline() : coefficients(6, 0.0) {};
    virtual ~Spline(){ coefficients.clear(); };
    virtual void sample(const double &time, Motion &motion) const {
      double t[6];
      pows(5, time, t);
      motion.position =
        t[0] * coefficients[0] +
        t[1] * coefficients[1] +
        t[2] * coefficients[2] +
        t[3] * coefficients[3] +
        t[4] * coefficients[4] +
        t[5] * coefficients[5];

      motion.velocity =
        1.0 * t[0] * coefficients[1] +
        2.0 * t[1] * coefficients[2] +
        3.0 * t[2] * coefficients[3] +
        4.0 * t[3] * coefficients[4] +
        5.0 * t[4] * coefficients[5];

      motion.acceleration =
        2.0  * t[0] * coefficients[2] +
        6.0  * t[1] * coefficients[3] +
        12.0 * t[2] * coefficients[4] +
        20.0 * t[3] * coefficients[5];
    }

    virtual void sampleWithTimeBounds(const double &duration,
                                      const double &time, Motion &motion) const {
      if (time < 0.0) {
        sample(0.0, motion);
        motion.velocity = 0.0;
        motion.acceleration = 0.0;
      } else if (time > duration) {
        sample(duration, motion);
        motion.velocity = 0.0;
        motion.acceleration = 0.0;
      } else {
        sample(time, motion);
      }
    }
  };

  struct Line : public Spline {
    Line(const Motion &start, const Motion &end, const double &duration) {
      coefficients[0] = start.position;
      if (duration == 0.0) coefficients[1] = 0.0;
      else coefficients[1] = (end.position - start.position) / duration;
    }
  };

  struct CubicSpline : public Spline {
    CubicSpline(const Motion &start, const Motion &end, const double &duration) {
      if (duration == 0.0) {
        coefficients[0] = end.position;
        coefficients[1] = end.velocity;
      } else {
        double t[4];
        pows(3, duration, t);
        coefficients[0] = start.position;
        coefficients[1] = start.velocity;
        coefficients[2] = (-3.0 * start.position +
                           3.0 * end.position -
                           2.0 * start.velocity * t[1] -
                           end.velocity * t[1]) / t[2];
        coefficients[3] = (2.0 * start.position -
                           2.0 * end.position +
                           start.velocity * t[1] +
                           end.velocity * t[1]) / t[3];
      }
    }
  };

  struct QuinticSpline : public Spline {
    QuinticSpline(const Motion &start, const Motion &end, const double &duration) {
      if (duration == 0.0) {
        coefficients[0] = end.position;
        coefficients[1] = end.velocity;
        coefficients[2] = 0.5 * end.acceleration;
      } else {
        double t[6];
        pows(5, duration, t);

        coefficients[0] = start.position;
        coefficients[1] = start.velocity;
        coefficients[2] = 0.5 * start.acceleration;
        coefficients[3] = (-20.0 * start.position +
                           20.0 * end.position -
                           3.0  * start.acceleration * t[2] +
                           end.acceleration * t[2] -
                           12.0 * start.velocity * t[1] -
                           8.0  * end.velocity * t[1]) / (2.0 * t[3]);
        coefficients[4] = ( 30.0 * start.position -
                            30.0 * end.position +
                            3.0  * start.acceleration * t[2] -
                            2.0  * end.acceleration * t[2] +
                            16.0 * start.velocity * t[1] +
                            14.0 * end.velocity * t[1]) / (2.0 * t[4]);
        coefficients[5] = (-12.0 * start.position +
                           12.0 * end.position -
                           start.acceleration * t[2] +
                           end.acceleration * t[2] -
                           6.0  * start.velocity * t[1] -
                           6.0  * end.velocity * t[1]) / (2.0*t[5]);
      }
    }
  };
} // namespace

#endif // MATH_SPLINE_H__
