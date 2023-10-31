//
// Created by hs on 23. 2. 9.
//

#include <robot_simulation/SimulControlPanelHigh.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

SimulControlPanelHigh::SimulControlPanelHigh()
    : mGaitPeriod((uint64_t)(ceil(GAIT_PERIOD/HIGH_CONTROL_dT)))
    , stand(MPC_HORIZON, mGaitPeriod,
            Vec4<int>(mGaitPeriod,mGaitPeriod,mGaitPeriod,mGaitPeriod),
            Vec4<int>(mGaitPeriod,mGaitPeriod,mGaitPeriod,mGaitPeriod))
    , trot(MPC_HORIZON, mGaitPeriod,
//            Vec4<int>(0,mGaitPeriod/2,mGaitPeriod/2,0),
        Vec4<int>(mGaitPeriod*1/4,mGaitPeriod*3/4,mGaitPeriod*3/4,mGaitPeriod*1/4),
            Vec4<int>(mGaitPeriod*3/4,mGaitPeriod*3/4,mGaitPeriod*3/4,mGaitPeriod*3/4))
{
}

void SimulControlPanelHigh::ControllerFunction()
{
    sharedMemory->gaitIteration = mIteration % mGaitPeriod;
    mIteration++;
    switch (sharedMemory->gaitState)
    {
        case STAND:
        {
            stand.getGaitTable();
            break;
        }
        case TROT:
        {
            trot.getGaitTable();
            break;
        }
        case CUSTOM_GAIT_1:
        {
            break;
        }
        case CUSTOM_GAIT_2:
        {
            break;
        }
        default:
        {
            break;
        }
    }
    if (sharedMemory->gaitChangeFlag)
    {
        switch(sharedCommand->gaitCommand)
        {
            case DESIRED_GAIT_STAND:
            {
                stand.getChangeGaitTable();
                if (sharedMemory->gaitIteration + 1 == mGaitPeriod)
                {
                    sharedMemory->gaitState = STAND;
                    sharedMemory->gaitChangeFlag = false;
                }
                break;
            }
            case DESIRED_GAIT_TROT:
            {
                sharedMemory->gaitIteration = mGaitPeriod - 1;\
                mIteration = mGaitPeriod -1;
                trot.getChangeGaitTable();
                if (sharedMemory->gaitIteration + 1 == mGaitPeriod)
                {
                    sharedMemory->gaitState = TROT;
                    sharedMemory->gaitChangeFlag = false;
                }
                break;
            }
            case DESIRED_GAIT_CUSTOM1:
            {
                break;
            }
            case DESIRED_GAIT_CUSTOM2:
            {
                break;
            }
            default:
            {
                break;
            }
        }
    }

    switch (sharedMemory->HighControlState)
    {
        case STATE_HIGH_CONTROL_STOP:
        {
            break;
        }
        case STATE_DEFAULT_CONTROL:
        {
            MPC.DoControl();
            break;
        }
        case STATE_HIGH_HOME_STAND_UP_READY:
        {
            MPC.InitUpTrajectory();

            sharedMemory->HighControlState = STATE_HIGH_DO_CONTROL;
            break;
        }
        case STATE_HIGH_HOME_STAND_DOWN_READY:
        {
            MPC.InitDownTrajectory();
            sharedMemory->HighControlState = STATE_HIGH_DO_CONTROL;
            break;
        }
        case STATE_HIGH_DO_CONTROL:
        {
            MPC.DoControl();
            sharedMemory->LowControlState = STATE_LOW_HOME_CONTROL;
            break;
        }
        default:
            break;
    }
}
