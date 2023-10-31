//
// Created by hs on 22. 10. 27.
//

#ifndef RAISIM_SIMULMAIN_HPP
#define RAISIM_SIMULMAIN_HPP

#include <iostream>
#include <QApplication>

#include <raisim/World.hpp>
#include <raisim/RaisimServer.hpp>

#include <camel-tools/ThreadGenerator.hpp>

#include <robot_util/RobotDescription.hpp>
#include <robot_util/SharedMemory.hpp>

#include <robot_simulation/SimulStateEstimator.hpp>
#include <robot_simulation/SimulControlPanel.hpp>
#include <robot_simulation/SimulControlPanelHigh.hpp>
#include <robot_simulation/SimulVisualizer.hpp>
#include <robot_simulation/SimulCommand.hpp>

void StartSimulation();
void* NRTCommandThread(void* arg);
void* RTControllerThread(void* arg);
void* RTStateEstimator(void* arg);
void* RTMPCThread(void* arg);

#endif //RAISIM_SIMULMAIN_HPP
