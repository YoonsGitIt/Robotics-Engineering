//
// Created by hs on 22. 10. 27.
//

#include <robot_simulation/SimulControlPanel.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

SimulControlPanel::SimulControlPanel(raisim::World* world, raisim::ArticulatedSystem* robot)
    : mWorld(world)
    , mRobot(robot)
    , mIteration(0)
    , mAlpha(0)
    , mRefTime(0)
    , bStandUp(false)
    , bStandDown(false)
{
    for (int idx = 0; idx < MOTOR_NUM; idx++)
    {
        mCalTorque[MOTOR_NUM] = 0;
    }
    mTorque.setZero();
}

void SimulControlPanel::ControllerFunction()
{
    sharedMemory->localTime = mIteration * LOW_CONTROL_dT;
    mIteration++;

    /// for alpha
    if (mAlpha > 1)
    {
        sharedMemory->HighControlState = STATE_DEFAULT_CONTROL;
        sharedMemory->LowControlState = STATE_LOW_CONTROL_START;
        bStandDown = false;
        bStandUp = false;
        mAlpha = 1;
    }
    else if (mAlpha < 0)
    {
        bStandDown = false;
        bStandUp = false;
        mAlpha = 0;
    }
    else
    {
        if (bStandUp)
        {
            mAlpha = (STAND_UP_TIME - mRefTime + sharedMemory->localTime) / STAND_UP_TIME;
        }
        if (bStandDown)
        {
            mAlpha = 1 - ((STAND_DOWN_TIME - mRefTime + sharedMemory->localTime) / STAND_DOWN_TIME);
        }
    }

    LowPDController.DoControl();

    switch (sharedMemory->LowControlState)
    {
    case STATE_LOW_CONTROL_STOP:
    {
        break;
    }
    case STATE_LOW_CONTROL_START:
    {
        LowController.DoControl();
        break;
    }
    case STATE_LOW_HOME_STAND_UP_READY:
    {
        PDcontrol.InitHomeStandUpTrajectory();
        mRefTime = sharedMemory->localTime + STAND_UP_TIME;
        bStandUp = true;
        break;
    }
    case STATE_LOW_HOME_STAND_DOWN_READY:
    {
        PDcontrol.InitHomeStandDownTrajectory();
        mRefTime = sharedMemory->localTime + STAND_DOWN_TIME;
        bStandDown = true;
    }
    case STATE_LOW_HOME_CONTROL:
    {
        //alpha*torque1+(1-alpha)*torque2
        LowController.DoControl();
        PDcontrol.DoHomeControl();
        for (int index = 0; index < MOTOR_NUM; index++)
        {
            mCalTorque[index] = mAlpha * LowController.GetTorque()[index] + (1 - mAlpha) * PDcontrol.GetTorque()[index];
            if (mCalTorque[index] > TORQUE_LIMIT)
            {
                mCalTorque[index] = TORQUE_LIMIT;
            }
            else if (mCalTorque[index] < -TORQUE_LIMIT)
            {
                mCalTorque[index] = -TORQUE_LIMIT;
            }
            sharedMemory->motorDesiredTorque[index] = mCalTorque[index];
        }
        break;
    }
    default:
        break;
    }
    if (sharedMemory->visualState == STATE_UPDATE_VISUAL)
    {
        integrateSimul();
    }
//    for (int idx=0; idx<ARM_MOTOR_NUM; idx++)
//    {
//        std::cout << sharedMemory->armMotorDesiredTorque[idx] << std::endl;
//    }
}

void SimulControlPanel::integrateSimul()
{
    for (int idx = 0; idx < MOTOR_NUM; idx++)
    {
        mTorque[idx + 6] = sharedMemory->motorDesiredTorque[idx];
    }
    for (int idx=0; idx<ARM_MOTOR_NUM; idx++)
    {
        mTorque[idx+MOTOR_NUM+6] = sharedMemory->armMotorDesiredTorque[idx];
    }
    mRobot->setGeneralizedForce(mTorque);
    mWorld->integrate();
}