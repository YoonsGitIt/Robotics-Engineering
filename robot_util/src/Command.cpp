#include <robot_util/Command.hpp>

extern pUI_COMMAND sharedCommand;
extern pSHM sharedMemory;

Command::Command()
: mLocalVelocity(0.0)
{
    initializeJoystick();
}

void Command::commandFunction()
{
    readJoystick();
    mappingJoystickCommand();
    if(sharedMemory->newCommand)
    {
        sharedMemory->newCommand = false;
        int incomingCommand = sharedCommand->userCommand;

        switch(incomingCommand)
        {
            case CAN_ON:
            {
                sharedMemory->canLFState = CAN_INIT;
                sharedMemory->canRFState = CAN_INIT;
                sharedMemory->canLBState = CAN_INIT;
                sharedMemory->canRBState = CAN_INIT;

                sharedMemory->newCommand = true;
                sharedCommand->userCommand = VISUAL_ON;
                break;
            }
            case VISUAL_ON:
            {
                sharedMemory->visualState = STATE_OPEN_RAISIM;
                sharedMemory->newCommand = true;
                sharedCommand->userCommand = MOTOR_ON;
                break;
            }
            case MOTOR_ON:
            {
                sharedMemory->HighControlState = STATE_HIGH_CONTROL_STOP;
                sharedMemory->canLFState = CAN_MOTOR_ON;
                sharedMemory->canRFState = CAN_MOTOR_ON;
                sharedMemory->canLBState = CAN_MOTOR_ON;
                sharedMemory->canRBState = CAN_MOTOR_ON;
                break;
            }
            case MOTOR_OFF:
            {
                sharedMemory->HighControlState = STATE_HIGH_CONTROL_STOP;
                sharedMemory->canLFState = CAN_MOTOR_OFF;
                sharedMemory->canRFState = CAN_MOTOR_OFF;
                sharedMemory->canLBState = CAN_MOTOR_OFF;
                sharedMemory->canRBState = CAN_MOTOR_OFF;
                break;
            }
            case CHANGE_GAIT_STAND:
            {
                sharedMemory->gaitChangeFlag = true;
                sharedCommand->gaitCommand = DESIRED_GAIT_STAND;
                break;
            }
            case CHANGE_GAIT_TROT:
            {
                sharedMemory->gaitChangeFlag = true;
                sharedCommand->gaitCommand = DESIRED_GAIT_TROT;
                break;
            }
            case CHANGE_GAIT_CUSTOM1:
            {
                break;
            }
            case CHANGE_GAIT_CUSTOM2:
            {
                break;
            }
            case HOME:
            {
                sharedMemory->HighControlState = STATE_HIGH_HOME_STAND_UP_READY;
                sharedMemory->LowControlState = STATE_LOW_HOME_STAND_UP_READY;
                sharedMemory->canLFState = CAN_SET_TORQUE;
                sharedMemory->canRFState = CAN_SET_TORQUE;
                sharedMemory->canLBState = CAN_SET_TORQUE;
                sharedMemory->canRBState = CAN_SET_TORQUE;
                break;
            }
            case READY:
            {
                sharedMemory->HighControlState = STATE_HIGH_HOME_STAND_DOWN_READY;
                sharedMemory->LowControlState = STATE_LOW_HOME_STAND_DOWN_READY;
                sharedMemory->canLFState = CAN_SET_TORQUE;
                sharedMemory->canRFState = CAN_SET_TORQUE;
                sharedMemory->canLBState = CAN_SET_TORQUE;
                sharedMemory->canRBState = CAN_SET_TORQUE;
                break;
            }
            case CUSTOM_1:
            {
                break;
            }
            case CUSTOM_2:
            {
                for(int i = 0 ; i < MOTOR_NUM ; i++)
                {
                    sharedMemory->motorDesiredTorque[i] = 0.0;
                }
                sharedMemory->canLFState = CAN_SET_TORQUE;
                sharedMemory->canRFState = CAN_SET_TORQUE;
                sharedMemory->canLBState = CAN_SET_TORQUE;
                sharedMemory->canRBState = CAN_SET_TORQUE;
                break;
            }
            default:
                break;
        }
    }
}

void Command::initializeJoystick()
{
    const char* ds4FilePath = "/dev/input/js0";
    mJoystickFd = -1;
    mJoystickNumOfAxis = 0;
    mJoystickNumOfButton = 0;
    if ((mJoystickFd = open(ds4FilePath, O_RDONLY)) < 0)
    {
        std::cerr << "Failed to open " << ds4FilePath << std::endl;
        return;
    }
    ioctl(mJoystickFd, JSIOCGAXES, &mJoystickNumOfAxis);
    ioctl(mJoystickFd, JSIOCGBUTTONS, &mJoystickNumOfButton);
    ioctl(mJoystickFd, JSIOCGNAME(80), &mJoystickName);
    mJoystickButton.resize(mJoystickNumOfButton, 0);
    mJoystickAxis.resize(mJoystickNumOfAxis, 0);
    std::cout << "Joystick: " << mJoystickName << std::endl
              << "  axis: " << mJoystickNumOfAxis << std::endl
              << "  buttons: " << mJoystickNumOfButton << std::endl;
    fcntl(mJoystickFd, F_SETFL, O_NONBLOCK);   // using non-blocking mode
}

void Command::readJoystick()
{
    js_event js;
    read(mJoystickFd, &js, sizeof(js_event));
    switch (js.type & ~JS_EVENT_INIT)
    {
        case JS_EVENT_AXIS:
            mJoystickAxis[(int)js.number] = js.value;
            break;
        case JS_EVENT_BUTTON:
            mJoystickButton[(int)js.number] = js.value;
            break;
    }
    mLocalVelocity = -(double)mJoystickAxis[1] / 100000.0;
    sharedMemory->baseDesiredVelocity[0] = mLocalVelocity * cos(sharedMemory->baseEulerPosition[2]);
    sharedMemory->baseDesiredVelocity[1] = mLocalVelocity * sin(sharedMemory->baseEulerPosition[2]);
    sharedMemory->baseDesiredEulerVelocity[2] = -(double)mJoystickAxis[3] / 75000.0;
}

void Command::printJoystickValue()
{
    std::cout << "axis/10000: ";
    for (size_t i(0); i < mJoystickAxis.size(); ++i)
    {
        std::cout << " " << std::setw(2) << (double)mJoystickAxis[i] / 10000;
    }
    std::cout << "  button: ";
    for (size_t i(0); i < mJoystickButton.size(); ++i)
    {
        std::cout << " " << (double)mJoystickButton[i];
    }
    std::cout << std::endl;
}

void Command::mappingJoystickCommand()
{
    switch(sharedMemory->FSMState)
    {
    case INITIAL:
    {
        if(mJoystickButton[9] == 1)
        {
            std::cout<<"[FSM] : INITIALIZE"<<std::endl;
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CAN_ON;
            sharedMemory->FSMState = FSM_READY;
        }
        break;
    }
    case VISUAL_CAN_READY:
    {
        /// escape motor off

        break;
    }
    case FSM_READY:
    {
        /// stand up
        if(mJoystickButton[7] == 1 && mJoystickAxis[7] < 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = HOME;
            sharedMemory->FSMState = FSM_STAND;
        }
        /// motor off
        if(mJoystickButton[6] == 1 && mJoystickButton[7] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_STAND:
    {
//        std::cout<<"[FSM_STAND]"<<std::endl;
        /// sit down
        if(mJoystickButton[7] == 1 && mJoystickAxis[7] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = READY;
            sharedMemory->FSMState = FSM_READY;
        }
        /// x,y axis trot
        if((sqrt(pow(sharedMemory->baseVelocity[0],2)+pow(sharedMemory->baseVelocity[1],2)) > 0.1) && abs(mLocalVelocity) > 0.0004)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT;
            sharedMemory->FSMState = FSM_TROT;
        }
        /// yaw trot
        if((abs(sharedMemory->baseEulerVelocity[2]) > 0.05) && abs(sharedMemory->baseDesiredEulerVelocity[2]) > 0.004)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT;
            sharedMemory->FSMState = FSM_TROT;
        }
/*
 *  /// trot change button
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] > 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_TROT;
            sharedMemory->FSMState = FSM_TROT;
        }
*/
        /// motor off
        if(mJoystickButton[6] == 1 && mJoystickButton[7] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = VISUAL_CAN_READY;
        }
        break;
    }
    case FSM_TROT:
    {
//        std::cout<<"[FSM_TROT]"<<std::endl;
        /// stand
        if(abs(mLocalVelocity) < 0.002 && abs(sharedMemory->baseEulerVelocity[2]) < 0.002)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_STAND;
        }
/*
 *  /// stand change button
        if(mJoystickButton[5] == 1 && mJoystickAxis[6] < 0)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = CHANGE_GAIT_STAND;
            sharedMemory->FSMState = FSM_STAND;
        }
*/
        /// motor off
        if(mJoystickButton[6] == 1 && mJoystickButton[7] == 1)
        {
            sharedMemory->newCommand = true;
            sharedCommand->userCommand = MOTOR_OFF;
            sharedMemory->FSMState = VISUAL_CAN_READY;
        }
        break;
    }
    default:
    {
        break;
    }

    }

}