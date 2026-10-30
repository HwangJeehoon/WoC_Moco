# OpenSim/Moco plugin used by moco_WoC_loop when
# opts.mocoObjectiveMode = 'minSpeedFixedEP' (EffortPropulsionIntegrator and
# MocoCoordinateAverageSpeedGoal).
#
# Build from MATLAB with build_osimFixedEffortPropulsion.m, or manually against
# the OpenSim installation that MATLAB uses:
#   cmake -S . -B build -DOpenSim_DIR="C:/OpenSim 4.5.2/cmake"
#   cmake --build build --config Release
# Use the same compiler as the OpenSim build (OpenSim 4.5.2 for Windows:
# Visual Studio 2022, x64, Release).
cmake_minimum_required(VERSION 3.15)
project(osimFixedEffortPropulsion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenSim 4.5 REQUIRED)

add_library(osimFixedEffortPropulsion SHARED
    osimFixedEffortPropulsionDLL.h
    EffortPropulsionIntegrator.h
    EffortPropulsionIntegrator.cpp
    MocoCoordinateAverageSpeedGoal.h
    MocoCoordinateAverageSpeedGoal.cpp
    RegisterTypes_osimFixedEffortPropulsion.cpp)
target_compile_definitions(osimFixedEffortPropulsion
    PRIVATE OSIMFIXEDEFFORTPROPULSION_EXPORTS)
target_link_libraries(osimFixedEffortPropulsion ${OpenSim_LIBRARIES})
