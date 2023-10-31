cmake_minimum_required(VERSION 3.5)
project(robot_util LANGUAGES CXX)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)

link_libraries(stdc++fs)

add_library(robot_util
        SHARED
        src/Command.cpp
        src/StateEstimator.cpp
        src/RobotMath.cpp
        src/Filter.cpp
        src/DataAnalysis.cpp)
target_link_libraries(robot_util
        PUBLIC
        raisim

)