# Copyright (C) 2017  iCub Facility, Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro <silvio.traversaro@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LICENSE


yarp_prepare_plugin(xsensmt TYPE yarp::dev::XsensMT
                            INCLUDE XsensMT.h
                            CATEGORY device
                            DEFAULT ON)

if(ENABLE_xsensmt)

  set(PLUGIN_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/XsensMT.cpp")
  set(PLUGIN_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/XsensMT.h")

  yarp_add_plugin(xsensmt ${PLUGIN_SOURCES} ${PLUGIN_HEADERS})

  target_link_libraries(xsensmt YARP::YARP_OS YARP::YARP_dev YARP::YARP_sig)
  target_link_libraries(xsensmt xsens_mt_software_suite)

  target_include_directories(xsensmt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${YARP_INCLUDE_DIRS})
  
  yarp_install(TARGETS xsensmt
         COMPONENT runtime
         LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
         ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR})
         
  yarp_install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/xsensmt.ini"  DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

  # Install the example file from Xsens
  add_executable(xsensExample exampleScanDevicesAndReceiveData.cpp)
  target_link_libraries(xsensExample PUBLIC xstypes xscommon xscontroller)
  install(TARGETS xsensExample RUNTIME DESTINATION bin)

endif()
