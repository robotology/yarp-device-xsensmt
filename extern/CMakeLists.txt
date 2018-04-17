# Copyright (C) 2017-2018  iCub Facility, Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LICENSE

# The code contained in this directory is imported directly from the src_cpp example 
# in the MT Software Suite, version 4.8.2, available from https://www.xsens.com/mt-software-suite/
# If you want to update the code to a new version of the MT Software Suite, run make in the mtsdk
# example directory and copy the relevant files from the src_example in this directory.

# Globbing should be avoided in general, but this is a library that we use directly as provided by Xsens 
file(GLOB xcommunication_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/xcommunication/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/xcommunication/*.c)
file(GLOB xstypes_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/xstypes/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/xstypes/*.c)
add_library(xsens_mt_software_suite STATIC deviceclass.cpp ${xcommunication_SRCS} ${xstypes_SRCS})
target_include_directories(xsens_mt_software_suite PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} 
                                                          ${CMAKE_CURRENT_SOURCE_DIR}/include 
                                                          ${CMAKE_CURRENT_SOURCE_DIR}/include/xcommunication 
                                                          ${CMAKE_CURRENT_SOURCE_DIR}/include/xsens) 
find_package (Threads)
target_link_libraries(xsens_mt_software_suite PUBLIC ${CMAKE_THREAD_LIBS_INIT} ${CMAKE_DL_LIBS})
