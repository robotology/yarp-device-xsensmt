# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.5)
project(yarp-device-xsensmt)
# Required by the XSens MT Software Suite
set (CMAKE_CXX_STANDARD 11)

# Find YARP and include the CMake code to compile plugins
find_package(YARP REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpPlugin)
include(YarpInstallationHelpers)

option(BUILD_SHARED_LIBS ON)
set(YARP_FORCE_DYNAMIC_PLUGINS TRUE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

yarp_configure_plugins_installation(yarp-device-xsensmt)

# Compile XSens-provided code as a static library 
# The code in this directory is directly extracted from the 
# src_cpp example provided in the MT Software Suite, see
# the comments in the contained CMakeLists.txt for more info
add_subdirectory(extern)

# Compile YARP device 
add_subdirectory(xsensmt)
