#ifndef AERO_COMMON_ANGLE_TO_STROKE_H_
#define AERO_COMMON_ANGLE_TO_STROKE_H_

#include <vector>
#include <algorithm>
#include <stdint.h>
#include <math.h>
#include "aero_hardware_interface/Angle2Stroke.hh"

namespace aero
{
  namespace common
  {

    struct dualJoint
    {
      float one;
      float two;
    };

    //////////////////////////////////////////////////
    void Angle2Stroke
    (std::vector<int16_t>& _strokes, const std::vector<double> _angles)
    {
      float rad2Deg = 180.0 / M_PI;
      float scale = 100.0;
      dualJoint right_wrist =
        WristRollPitchTable(rad2Deg * r_wrist_p_joint,
                            rad2Deg * r_wrist_r_joint);
      dualJoint left_wrist =
        WristRollPitchTable(rad2Deg * l_wrist_p_joint,
                            -rad2Deg * l_wrist_r_joint);
      dualJoint waist =
        WaistRollPitchTable(-rad2Deg * waist_r_joint,
                            rad2Deg * waist_p_joint);
      dualJoint neck =
        NeckRollPitchTable(rad2Deg * neck_r_joint,
                           rad2Deg * neck_p_joint);

      // ros_order -> can_order
      meta = scale * rad2Deg * neck_y_joint;
      meta = scale * neck.one;
      meta = scale * neck.two;

      meta =
        scale * ShoulderPitchTable(-rad2Deg * r_shoulder_p_joint);
      meta =
        scale * ShoulderRollTable(-rad2Deg * r_shoulder_r_joint);
      meta = -scale * rad2Deg * r_shoulder_y_joint;
      meta = scale * ElbowPitchTable(-rad2Deg * r_elbow_joint);
      meta = -scale * rad2Deg * r_wrist_y_joint;
      meta = scale * right_wrist.one;
      meta = scale * right_wrist.two;
      meta = -scale * (rad2Deg * r_indexbase_joint + 65.0) * 0.12;
      meta = -scale * rad2Deg * r_hand_y_joint;
      meta = scale * (rad2Deg * r_thumb_joint - 15) * 0.09;

      meta =
        scale * ShoulderPitchTable(-rad2Deg * l_shoulder_p_joint);
      meta =
        scale * ShoulderRollTable(rad2Deg * l_shoulder_r_joint);
      meta = -scale * rad2Deg * l_shoulder_y_joint;
      meta = scale * ElbowPitchTable(-rad2Deg * l_elbow_joint);
      meta = -scale * rad2Deg * l_wrist_y_joint;
      meta = scale * left_wrist.one;
      meta = scale * left_wrist.two;
      meta = scale * (rad2Deg * l_indexbase_joint - 65.0) * 0.12;
      meta = -scale * rad2Deg * l_hand_y_joint;
      meta = scale * (rad2Deg * l_thumb_joint + 50) * 0.18;

      meta = scale * waist.two;
      meta = scale * waist.one;
      meta = scale * rad2Deg * waist_y_joint;

      meta = scale * LegTable(rad2Deg * ankle_joint);
      meta = scale * LegTable(- rad2Deg * knee_joint);
    };

  }
}

#endif
