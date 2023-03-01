# Copyright (C) 2017-2023 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LICENSE

# The code contained in this directory is imported directly from the src_cpp example 
# in the MT Software Suite, version 2022.0.0, available from https://www.xsens.com/mt-software-suite/
# If you want to update the code to a new version of the MT Software Suite, run make in the mtsdk
# example directory and copy the xspublic files from the public directory in this directory.
# Furthermore, you should update the CMake scripts options by inspecting the xspublic's Makefiles


# Globbing should be avoided in general, but this is a library that we use directly as provided by Xsens 
file(GLOB xstypes_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xstypes/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xstypes/*.c)
# See EXCLUDE argument in xstypes' Makefile
list(FILTER xstypes_SRCS EXCLUDE REGEX "xstypesdynlib")
add_library(xstypes STATIC ${xstypes_SRCS})
target_include_directories(xstypes PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/xspublic)
target_include_directories(xstypes PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xstypes)


file(GLOB xscommon_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xscommon/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xscommon/*.c)
add_library(xscommon STATIC ${xscommon_SRCS})
target_include_directories(xscommon PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/xspublic)
target_include_directories(xscommon PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xscommon)
target_compile_options(xscommon PRIVATE "SHELL:-include xscommon_config.h" "SHELL:-include xstypes/xsens_compat.h")
target_link_libraries(xscommon PUBLIC xstypes)


file(GLOB xscontroller_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xscontroller/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xscontroller/*.c)
add_library(xscontroller STATIC ${xscontroller_SRCS})
target_include_directories(xscontroller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/xspublic)
target_include_directories(xscontroller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/xspublic/xscontroller)
target_compile_options(xscontroller PRIVATE "SHELL:-include xscontrollerconfig.h")
target_compile_definitions(xscontroller PRIVATE -DHAVE_JOURNALLER)
target_link_libraries(xscontroller PUBLIC xstypes xscommon)

find_package (Threads)

add_library(xsens_mt_software_suite INTERFACE)
target_link_libraries(xsens_mt_software_suite INTERFACE xstypes xscommon xscontroller)
