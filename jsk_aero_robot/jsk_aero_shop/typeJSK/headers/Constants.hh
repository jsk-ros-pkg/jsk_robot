#ifndef AERO_CONTROLLER_CONSTANTS_H_
#define AERO_CONTROLLER_CONSTANTS_H_

namespace aero
{
  namespace controller
  {
    // id: upper = 1, lower = 2
    const static uint8_t ID_UPPER = 1;
    const static uint8_t ID_LOWER = 2;

    // whole: 34DOF
    //  upper: 22DOF
    //   neck: 3DOF
    //   arm: 8DOF (including hand) * 2
    //    shoulder: 2DOF * 2
    //    elbow: 2DOF * 2
    //    wrist: 3DOF * 2
    //    hand: 1DOF * 2
    //   waist: 3DOF
    //  lower : 0DOF
    //    (wheel: 1DOF * 4)

    const static size_t AERO_DOF = 28;
    const static size_t AERO_DOF_UPPER = 26;
    const static size_t AERO_DOF_LOWER = 2;
    const static size_t AERO_DOF_WHEEL = 4;

    // joint index in stroke vector
    // UPPER:
    const static size_t CAN_NECK_Y = 0;
    const static size_t CAN_NECK_RIGHT = 1;
    const static size_t CAN_NECK_LEFT = 2;

    const static size_t CAN_R_SHOULDER_P = 3;
    const static size_t CAN_R_SHOULDER_R = 4;
    const static size_t CAN_R_ELBOW_Y = 5;
    const static size_t CAN_R_ELBOW_P = 6;
    const static size_t CAN_R_WRIST_Y = 7;
    const static size_t CAN_R_WRIST_TOP = 8;
    const static size_t CAN_R_WRIST_BOTTOM = 9;
    const static size_t CAN_R_INDEX = 10;
    const static size_t CAN_R_HAND_Y = 11;
    const static size_t CAN_R_THUMB = 12;

    const static size_t CAN_L_SHOULDER_P = 13;
    const static size_t CAN_L_SHOULDER_R = 14;
    const static size_t CAN_L_ELBOW_Y = 15;
    const static size_t CAN_L_ELBOW_P = 16;
    const static size_t CAN_L_WRIST_Y = 17;
    const static size_t CAN_L_WRIST_TOP = 18;
    const static size_t CAN_L_WRIST_BOTTOM = 19;
    const static size_t CAN_L_INDEX = 20;
    const static size_t CAN_L_HAND_Y = 21;
    const static size_t CAN_L_THUMB = 22;

    const static size_t CAN_WAIST_RIGHT = 23;
    const static size_t CAN_WAIST_LEFT = 24;
    const static size_t CAN_WAIST_Y = 25;

    // LOWER:
    const static size_t CAN_DOWN = 0;
    const static size_t CAN_UP = 1;

    // WHEEL:
    const static size_t CAN_FRONT_R_WHEEL = 0;
    const static size_t CAN_REAR_R_WHEEL = 1;
    const static size_t CAN_FRONT_L_WHEEL = 2;
    const static size_t CAN_REAR_L_WHEEL = 3;


    // joint index in raw vector (as int16_t)
    // UPPER: ID = 1
    const static size_t RAW_NECK_Y = 0;
    const static size_t RAW_NECK_RIGHT = 1;
    const static size_t RAW_NECK_LEFT = 2;
    const static size_t RAW_R_SHOULDER_P = 3;
    const static size_t RAW_R_SHOULDER_R = 4;
    const static size_t RAW_R_ELBOW_Y = 5;
    const static size_t RAW_R_ELBOW_P = 6;
    const static size_t RAW_R_WRIST_Y = 7;
    const static size_t RAW_R_WRIST_TOP = 8;
    const static size_t RAW_R_WRIST_BOTTOM = 9;
    const static size_t RAW_WAIST_RIGHT = 10;
    const static size_t RAW_R_INDEX = 11;
    const static size_t RAW_R_HAND_Y = 12;
    const static size_t RAW_R_THUMB = 13;
    // 12 - 15: Force Sensor (uint8_t * 6, 2bytes N/A)
    const static size_t RAW_WAIST_Y = 15;
    // 17 - 18: N/A
    const static size_t RAW_L_SHOULDER_P = 18;
    const static size_t RAW_L_SHOULDER_R = 19;
    const static size_t RAW_L_ELBOW_Y = 20;
    const static size_t RAW_L_ELBOW_P = 21;
    const static size_t RAW_L_WRIST_Y = 22;
    const static size_t RAW_L_WRIST_TOP = 23;
    const static size_t RAW_L_WRIST_BOTTOM = 24;
    const static size_t RAW_WAIST_LEFT = 25;
    const static size_t RAW_L_INDEX = 28;
    const static size_t RAW_L_HAND_Y = 27;
    const static size_t RAW_L_THUMB = 26;
    // 28 - 31: Force Sensor (uint8_t * 6, 2bytes N/A)
    // 32 - 34: N/A

    // LOWER :
    const static size_t RAW_DOWN = 0;
    const static size_t RAW_UP = 1;
    const static size_t RAW_FRONT_R_WHEEL = 2;
    const static size_t RAW_REAR_R_WHEEL = 3;
    // 12 - 15: N/A
    const static size_t RAW_FRONT_L_WHEEL = 4;
    const static size_t RAW_REAR_L_WHEEL = 5;
    // 28 - 31: N/A
    // 32 - 34: IMU (uint8_t * 6)

    // offsets
    // UPPER:
    const static size_t OFFSET_R_SHOULDER_R = 1119;
    //const static size_t OFFSET_R_HAND = 900;
    const static size_t OFFSET_L_SHOULDER_R = 1119;
    //const static size_t OFFSET_L_HAND = 900;

    // LOWER:

    // sensor index (as int8_t)
    // UPPER:
    const static size_t RIGHT_HAND_SENSOR_FX = 30;
    const static size_t RIGHT_HAND_SENSOR_FY = 31;
    const static size_t RIGHT_HAND_SENSOR_FZ = 32;
    const static size_t RIGHT_HAND_SENSOR_RX = 33;
    const static size_t RIGHT_HAND_SENSOR_RY = 34;
    const static size_t RIGHT_HAND_SENSOR_RZ = 35;

    const static size_t LEFT_HAND_SENSOR_FX = 62;
    const static size_t LEFT_HAND_SENSOR_FY = 63;
    const static size_t LEFT_HAND_SENSOR_FZ = 64;
    const static size_t LEFT_HAND_SENSOR_RX = 65;
    const static size_t LEFT_HAND_SENSOR_RY = 66;
    const static size_t LEFT_HAND_SENSOR_RZ = 67;

    // LOWER :

  }
}

#endif
