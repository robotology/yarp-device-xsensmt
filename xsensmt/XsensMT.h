/*
 * Copyright (C) 2017 iCub Facility,  Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#ifndef XSENS_MT_YARP_DRIVER
#define XSENS_MT_YARP_DRIVER

#include "deviceclass.h"


#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace yarp 
{
namespace dev 
{
    class XsensMT;
}
}


/**
 * 
* \section xsensmt Description of input parameters
* \brief Device that implements IGenericSensor for Xsens MT* devices using the Xsens MT software suite.
*
* This device is based on the self-contained src_cpp example from the Xsens MT software suite. 
*
* Parameters accepted in the config argument of the open method:
* | Parameter name | Type    | Units | Default Value | Required  | Description   | Notes |
* |:--------------:|:-------:|:-----:|:-------------:|:--------: |:-------------:|:-----:|
* | serial         | string  |       | /dev/ttyUSB0  | No        | File name of the serial device to which to connect.  | |
* | baud           | int     |       | 115200        | No        | Baud rate used by the serial communication." | | 
* | timeout        | double  |seconds| 0.1           | No        | If the device is not receiving any sensor measure for timeout seconds, it will start reporting an error. | - |
*
**/
class yarp::dev::XsensMT : public yarp::dev::IGenericSensor, 
                           public yarp::dev::IPreciselyTimed, 
                           public yarp::dev::DeviceDriver
{
public:
    XsensMT();
    ~XsensMT();
    
    // IGenericSensor interface.
    virtual bool read(yarp::sig::Vector &out);
    virtual bool getChannels(int *nc);
    virtual bool open(yarp::os::Searchable &config);
    virtual bool calibrate(int ch, double v);
    virtual bool close();
    
    // Function executed by m_sensorThread
    void sensorReadLoop();

    virtual yarp::os::Stamp getLastInputStamp();
    
    // Thread running the sensorReadLoop method 
    std::thread m_sensorThread;

private:
    int m_nchannels{12};
    double m_timeoutInSecond{0.1};
    
    // True if the close method has been called at least once 
    std::atomic<bool> m_isDeviceClosing{false};
    
    // True if a valid sensor measurement has been received in the last m_sensorTimeout seconds 
    bool m_isSensorMeasurementAvailable{false};
    
    // Interface exposed by the Xsens MT Software suite
    DeviceClass m_xsensDevice;
    XsPortInfo m_portInfo;
   
    yarp::os::Stamp  m_lastReadStamp;

    
    // Mutex protecting m_sensorBuffer, m_lastStamp
    std::mutex m_bufferMutex;
    yarp::sig::Vector m_sensorBuffer;
    yarp::os::Stamp  m_lastStamp;
};


#endif
