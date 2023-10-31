//
// Created by hs on 23. 2. 9.
//

#ifndef RAISIM_SIMULCONTROLPANELHIGH_HPP
#define RAISIM_SIMULCONTROLPANELHIGH_HPP

#include <robot_util/SharedMemory.hpp>
#include <robot_util/RobotDescription.hpp>
#include <robot_util/EigenTypes.hpp>


class SimulControlPanelHigh{
public:
    SimulControlPanelHigh();

    void ControllerFunction();
private:
    uint64_t mGaitPeriod;
    uint64_t mIteration;

    MPCController MPC;
    OffsetGait stand, trot;
};

#endif //RAISIM_SIMULCONTROLPANELHIGH_HPP
