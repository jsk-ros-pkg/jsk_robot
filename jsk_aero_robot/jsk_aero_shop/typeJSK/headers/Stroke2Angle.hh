#ifndef AERO_COMMON_STROKE_TO_ANGLE_H_
#define AERO_COMMON_STROKE_TO_ANGLE_H_

#include <vector>
#include <algorithm>
#include <stdint.h>
#include <math.h>
#include "aero_hardware_interface/Stroke2Angle.hh"

namespace aero
{
  namespace common
  {

    struct S2AData
    {
      int angle;
      float stroke;
      float range;
    };

    //////////////////////////////////////////////////
    void Stroke2Angle
    (std::vector<double>& _angles, const std::vector<int16_t> _strokes)
    {
      float scale = 0.01;
      float left_wrist_roll_stroke =
        (scale * can_l_wrist_top + scale * can_l_wrist_bottom) * 0.5;
      float right_wrist_roll_stroke =
        (scale * can_r_wrist_top + scale * can_r_wrist_bottom) * 0.5;
      float waist_pitch_stroke =
        (scale * can_waist_right + scale * can_waist_left) * 0.5;
      float neck_pitch_stroke =
        (scale * can_neck_right + scale * can_neck_left) * 0.5;
      float deg2Rad = M_PI / 180.0;
      float knee_angle  = - deg2Rad * LegInvTable(scale * can_up);
      float ankle_angle = deg2Rad * LegInvTable(scale * can_down);

      // can_order -> ros_order
      meta =
        deg2Rad * scale * can_waist_y;
      meta =
        deg2Rad * WaistPitchInvTable(waist_pitch_stroke);
      meta =
        deg2Rad * WaistRollInvTable(scale * can_waist_right - waist_pitch_stroke);

      meta =
        -deg2Rad * ShoulderPitchInvTable(scale * can_l_shoulder_p);
      meta =
        deg2Rad * ShoulderRollInvTable(scale * can_l_shoulder_r);
      meta =
        -deg2Rad * scale * can_l_elbow_y;
      meta =
        -deg2Rad * ElbowPitchInvTable(scale * can_l_elbow_p);
      meta =
        -deg2Rad * scale * can_l_wrist_y;
      meta =
        deg2Rad * WristPitchInvTable(scale * can_l_wrist_top - left_wrist_roll_stroke);
      meta =
        -deg2Rad * WristRollInvTable(left_wrist_roll_stroke);
      meta =
        -deg2Rad * scale * can_l_hand_y;
      meta =
        -deg2Rad * (scale * can_l_thumb * 5.556 - 50.0);
      meta = 0;
      meta = 0;
      meta =
        deg2Rad * (scale * can_l_thumb * 5.556 - 50.0);

      meta =
        deg2Rad * scale * can_neck_y;
      meta =
        deg2Rad * NeckPitchInvTable(neck_pitch_stroke);
      meta =
        -deg2Rad * NeckRollInvTable(scale * can_neck_right - neck_pitch_stroke);

      meta =
        -deg2Rad * ShoulderPitchInvTable(scale * can_r_shoulder_p);
      meta =
        -deg2Rad * ShoulderRollInvTable(scale * can_r_shoulder_r);
      meta =
        -deg2Rad * scale * can_r_elbow_y;
      meta =
        -deg2Rad * ElbowPitchInvTable(scale * can_r_elbow_p);
      meta =
        -deg2Rad * scale * can_r_wrist_y;
      meta =
        deg2Rad * WristPitchInvTable(scale * can_r_wrist_top - right_wrist_roll_stroke);
      meta =
        deg2Rad * WristRollInvTable(right_wrist_roll_stroke);
      meta =
        -deg2Rad * scale * can_r_hand_y;
      meta =
        -deg2Rad * (scale * can_r_index * 8.333 + 65.0);
      meta = 0;
      meta = 0;
      meta =
        deg2Rad * (scale * can_r_thumb * 11.111 + 15.0);

      meta = knee_angle;
      meta = ankle_angle;
    };

  }
}

#endif
