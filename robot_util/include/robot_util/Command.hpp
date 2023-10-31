#ifndef RAISIM_COMMAND_H
#define RAISIM_COMMAND_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include "RobotDescription.hpp"
#include "SharedMemory.hpp"

class Command {
public:
    Command();

    void commandFunction();
private:
    void initializeJoystick();
    void readJoystick();
    void printJoystickValue();
    void mappingJoystickCommand();

private:
    int mJoystickFd;
    int mJoystickNumOfAxis;
    int mJoystickNumOfButton;
    double mLocalVelocity;
    char mJoystickName[80];
    std::vector<char> mJoystickButton;
    std::vector<int> mJoystickAxis;
};


#endif //RAISIM_COMMAND_H
