cmake_minimum_required(VERSION 3.5)
project(visualizer LANGUAGES CXX)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} STATIC
        src/RobotVisualization.cpp
)

#add_library(visualizer::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
        raisim
)