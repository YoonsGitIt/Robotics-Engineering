//
// Created by hs on 22. 10. 28.
//

#ifndef RAISIM_SIMULCOMMAND_HPP
#define RAISIM_SIMULCOMMAND_HPP

#include <iostream>
#include <iomanip>
#include <vector>
#include <cstdio>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <robot_util/SharedMemory.hpp>
#include <robot_util/RobotDescription.hpp>

class SimulCommand {
public:
    SimulCommand();
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

#endif //RAISIM_SIMULCOMMAND_HPP
