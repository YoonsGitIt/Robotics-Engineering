CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
project(robotics-engineering LANGUAGES CXX)

set(RAISIM_PATH "$ENV{HOME}/Library/raisimLib/raisim/linux")
include_directories(${RAISIM_PATH}/include)
link_directories(${RAISIM_PATH}/lib)

ADD_COMPILE_DEFINITIONS (URDF_RSC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/urdf/")

find_package(Eigen3 REQUIRED HINTS ${Eigen3_HINT})

add_subdirectory(visualizer)
add_subdirectory(robot_simulation)
add_subdirectory(robot_util)

include_directories(
        ${EIGEN3_INCLUDE_DIRS}
        ${visualizer_SOURCE_DIR}/include
        ${robot_simulation_SOURCE_DIR}/include
)

add_executable(robotic-engineering robot_demo/RobotMain.cpp

)
