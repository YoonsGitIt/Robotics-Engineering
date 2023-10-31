//
// Created by camel on 22. 9. 21.
//

#ifndef RAISIM_ROBOTDESCRIPTION_H
#define RAISIM_ROBOTDESCRIPTION_H

#include "EigenTypes.hpp"

constexpr int MOTOR_NUM = 12;
constexpr int MOTOR_NUM_PER_CAN = 3;
constexpr int ARM_MOTOR_NUM = 5;

constexpr double GRAVITY = -9.81;
constexpr double BODYMASS = 18.401;// with head : +0.275
constexpr double BODY_INERTIA[3] = { 0.104611, 0.241354, 0.306108 };// with head : {0.106175, 0.244961, 0.310683}
constexpr double TORQUE_LIMIT = 15;
constexpr double STAND_UP_TIME = 5.0;
constexpr double STAND_DOWN_TIME = 2.0;

///ARM Length
constexpr double LEN_ARM_L1 = 1.0;
constexpr double LEN_ARM_L2 = 2.5;
constexpr double LEN_ARM_L3 = 1.0;
constexpr double LEN_ARM_L4 = 1.0;
constexpr double LEN_ARM_L5 = 2.5;
constexpr double LEN_ARM_L6 = 0.5;

/// LF is standard
/// This exist for transform matrix
constexpr double POS_BAS_HIP_X = 0.185;
constexpr double POS_BAS_HIP_Y = 0.065;
constexpr double POS_HIP_THI_X = 0.07;
constexpr double POS_HIP_THI_Y = -0.003;
constexpr double POS_THI_CAL_Y = 0.084;
constexpr double POS_THI_CAL_Z = -0.23;
constexpr double POS_CAL_FOO_Z = -0.23;

/// This exist for jacobian matrix
constexpr double LEN_HIP = POS_HIP_THI_Y+POS_THI_CAL_Y;
constexpr double LEN_THI = -POS_THI_CAL_Z;
constexpr double LEN_CAL = -POS_CAL_FOO_Z;

// TODO have to check
constexpr double HIP_X_POS = POS_BAS_HIP_X + POS_HIP_THI_X;
constexpr double HIP_Y_POS = POS_BAS_HIP_Y + POS_HIP_THI_Y;
constexpr double SHOULD_X_POS = HIP_X_POS;
constexpr double SHOULD_Y_POS = POS_BAS_HIP_Y + POS_HIP_THI_Y + POS_THI_CAL_Y;


//TODO: change data structure of motor ID to keymap
constexpr int MOTOR_LFHR_ID = 0x141;
constexpr int MOTOR_LFHP_ID = 0x142;
constexpr int MOTOR_LFKP_ID = 0x143;

constexpr int MOTOR_RFHR_ID = 0x141;
constexpr int MOTOR_RFHP_ID = 0x142;
constexpr int MOTOR_RFKP_ID = 0x143;

constexpr int MOTOR_LBHR_ID = 0x141;
constexpr int MOTOR_LBHP_ID = 0x142;
constexpr int MOTOR_LBKP_ID = 0x143;

constexpr int MOTOR_RBHR_ID = 0x141;
constexpr int MOTOR_RBHP_ID = 0x142;
constexpr int MOTOR_RBKP_ID = 0x143;

constexpr double LFHR_POS_OFFSET = -0.4350;
constexpr double LFHP_POS_OFFSET = 1.970214;
constexpr double LFKP_POS_OFFSET = -3.395126;

constexpr double RFHR_POS_OFFSET = -0.1995;
constexpr double RFHP_POS_OFFSET = -2.734114;
constexpr double RFKP_POS_OFFSET = 2.694026;

constexpr double LBHR_POS_OFFSET = -0.2545;
constexpr double LBHP_POS_OFFSET = 2.010014;
constexpr double LBKP_POS_OFFSET = -3.404726;

constexpr double RBHR_POS_OFFSET = -0.4068;
constexpr double RBHP_POS_OFFSET = -2.660714;
constexpr double RBKP_POS_OFFSET = 2.604426;

//constexpr double LUMPED_MASS = 0.0;

enum LEG_INDEX
{
    LF_IDX = 0,
    RF_IDX,
    LB_IDX,
    RB_IDX
};

enum MOTOR_INDEX
{
    LFHR_IDX = 0,
    LFHP_IDX,
    LFKP_IDX,
    RFHR_IDX,
    RFHP_IDX,
    RFKP_IDX,
    LBHR_IDX,
    LBHP_IDX,
    LBKP_IDX,
    RBHR_IDX,
    RBHP_IDX,
    RBKP_IDX
};

enum ARM_JOINT
{
    ARM_JOINT_1 = 0,
    ARM_JOINT_2,
    ARM_JOINT_3,
    ARM_JOINT_4,
    ARM_JOINT_5,
    ARM_JOINT_6,
};

enum COMMAND
{
    NO_ACT,
    CAN_ON,
    VISUAL_ON,
    MOTOR_ON,
    MOTOR_OFF,
    HOME,
    READY,
    CUSTOM_1,
    CUSTOM_2,
    SIMUL_ON,
    SIMUL_OFF,
    CHANGE_GAIT_STAND,
    CHANGE_GAIT_TROT,
    CHANGE_GAIT_CUSTOM1,
    CHANGE_GAIT_CUSTOM2
};

enum HIGH_CONTROL_STATE
{
    STATE_HIGH_CONTROL_STOP,
    STATE_DEFAULT_CONTROL,
    STATE_HIGH_HOME_STAND_UP_READY,
    STATE_HIGH_HOME_STAND_DOWN_READY,
    STATE_HIGH_DO_CONTROL,
};

enum LOW_CONTROL_STATE
{
    STATE_LOW_CONTROL_STOP,
    STATE_LOW_CONTROL_START,
    STATE_LOW_HOME_STAND_UP_READY,
    STATE_LOW_HOME_STAND_DOWN_READY,
    STATE_LOW_HOME_CONTROL,
};

enum CAN_STATE
{
    CAN_NO_ACT,
    CAN_INIT,
    CAN_MOTOR_ON,
    CAN_MOTOR_OFF,
    CAN_SET_TORQUE,
    CAN_READ_ERROR
};

enum VISUAL_STATE
{
    STATE_VISUAL_STOP,
    STATE_OPEN_RAISIM,
    STATE_UPDATE_VISUAL
};

enum GAIT_TYPE
{
    STAND = 0,
    TROT,
    CUSTOM_GAIT_1,
    CUSTOM_GAIT_2
};

enum GAIT_COMMAND
{
    DESIRED_GAIT_STAND,
    DESIRED_GAIT_TROT,
    DESIRED_GAIT_CUSTOM1,
    DESIRED_GAIT_CUSTOM2
};

enum FSM
{
    INITIAL,
    VISUAL_CAN_READY,
    FSM_READY,
    FSM_STAND,
    FSM_TROT
};

enum Axes
{
    ROLL,
    PITCH,
    YAW
};


#endif //RAISIM_ROBOTDESCRIPTION_H
