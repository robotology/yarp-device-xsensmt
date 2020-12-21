# Copyright (C) 2017-2020 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LICENSE

# The code contained in this directory is imported directly from the src_cpp example 
# in the MT Software Suite, version 2020.0.2, available from https://www.xsens.com/mt-software-suite/
# If you want to update the code to a new version of the MT Software Suite, run make in the mtsdk
# example directory and copy the xspublic files from the public directory in this directory.

# Globbing should be avoided in general, but this is a library that we use directly as provided by Xsens 
file(GLOB xscommon_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xscommon/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xscommon/*.c)
file(GLOB xstypes_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xstypes/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xstypes/*.c)
file(GLOB xscontroller_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xscontroller/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xscontroller/*.c)

add_library(xsens_mt_software_suite STATIC ${xscommon_SRCS} ${xstypes_SRCS} ${xscontroller_SRCS})
target_include_directories(xsens_mt_software_suite PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} 
                                                          ${CMAKE_CURRENT_SOURCE_DIR}/xspublic
                                                          ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xscommon
                                                          ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xscontroller
                                                          ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xstypes)
find_package (Threads)
target_link_libraries(xsens_mt_software_suite)
