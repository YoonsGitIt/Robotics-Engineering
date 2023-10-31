cmake_minimum_required(VERSION 3.5)
project(robot_simulation LANGUAGES CXX)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}

        ${robot_util_SOURCE_DIR}/include
#
#        ${ControlUtils_SOURCE_DIR}/include
#        ${PDcontroller_SOURCE_DIR}/include
#        ${LowController_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME}
        STATIC
        src/SimulCommand.cpp
        src/SimulControlPanel.cpp
        src/SimulControlPanelHigh.cpp
        src/SimulMain.cpp
        src/SimulStateEstimator.cpp
        src/SimulVisualizer.cpp
        )

target_link_libraries(${PROJECT_NAME}
        raisim

#        robot_controller::ControlUtils
#        robot_controller::LowController
#        robot_controller::PDcontroller
        robot_util
)