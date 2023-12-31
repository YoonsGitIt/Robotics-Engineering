//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_SIMULCONTROLPANEL_HPP
#define RAISIM_SIMULCONTROLPANEL_HPP

#include <stdint.h>

#include <raisim/World.hpp>

#include <robot_util/SharedMemory.hpp>
#include <robot_util/RobotDescription.hpp>
#include <robot_util/EigenTypes.hpp>

#include <LowController/LowPDcontrol.hpp>
#include <LowController/LowWholeBodyPDController.hpp>
#include <ControlUtils/Gait.hpp>

#include <PDcontroller/JointPDController.hpp>

class SimulControlPanel{
public:
    SimulControlPanel(raisim::World* world, raisim::ArticulatedSystem* robot);

    void ControllerFunction();
private:
    void integrateSimul();
private:
    raisim::World* mWorld;
    raisim::ArticulatedSystem* mRobot;
    raisim::VecDyn mTorque = raisim::VecDyn(23);

    LowPDcontrol LowPDController;
    LowWholeBodyPDController LowController;
    JointPDController PDcontrol;

    uint64_t mIteration;
    double mAlpha;
    double mCalTorque[MOTOR_NUM];
    double mRefTime;
    bool bStandUp;
    bool bStandDown;

};

#endif //RAISIM_SIMULCONTROLPANEL_HPP
