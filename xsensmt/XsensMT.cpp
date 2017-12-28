/*
 * Copyright (C) 2017 iCub Facility,  Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include <yarp/os/LogStream.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>

#include <chrono>
#include <iostream>
#include <string>

#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/legacydatapacket.h>
#include <xsens/xsdatapacket.h>

#include "XsensMT.h"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

#define XSENS_PI             3.14159265358979323846264338328
#define CTRL_RAD2DEG    (180.0/XSENS_PI)
#define CTRL_DEG2RAD    (XSENS_PI/180.0)

XsensMT::XsensMT()
{
    m_sensorBuffer.resize(m_nchannels);
    m_sensorBuffer.zero();
}

XsensMT::~XsensMT()
{
    close();
}

bool XsensMT::read(Vector &out)
{
    bool ret = false;
    
    if (m_isSensorMeasurementAvailable)
    { 
        std::unique_lock<std::mutex> lock(m_bufferMutex);
        for (int i = 0; i < m_nchannels; i++)
        {
            out[i]=m_sensorBuffer[i];
        }
        m_lastReadStamp = m_lastStamp; 
        //std::cerr << "Read data " << out.toString() << std::endl;
        lock.unlock();

        ret=true;
    }
    else
    {
        ret=false;
    }
    
    double now = yarp::os::SystemClock::nowSystem();
    if (now-m_lastReadStamp.getTime() > m_timeoutInSecond)
    {
        yError("xsensmt: Sensor timeout, no sensor measurement received in the last %lf seconds.", now-m_lastReadStamp.getTime());
        m_isSensorMeasurementAvailable = false;
    }
    
    return ret;
}

bool XsensMT::getChannels(int *nc)
{
    *nc=m_nchannels;
    return true;
}

bool XsensMT::calibrate(int ch, double v)
{
    return false;
}

bool XsensMT::open(yarp::os::Searchable &config)
{
    std::string comPortString = config.check("serial", yarp::os::Value("/dev/ttyUSB0"), "File of the serial device.").asString().c_str();
    int baudRate = config.check("baud", yarp::os::Value(115200), "Baud rate used by the serial communication.").asInt();
    m_timeoutInSecond = config.check("timeout", yarp::os::Value(0.1), "Timeout of the driver").asDouble();
    
    m_portInfo = XsPortInfo(comPortString, XsBaud::numericToRate(baudRate));
    
    yInfo("xsensmt: Opening serial port %s with baud rate %d.", comPortString.c_str(), baudRate);
    if (!m_xsensDevice.openPort(m_portInfo))
    {
        yError("xsensmt: Could not open serial port.");
        return false;
    }

    yInfo("xsensmt: Putting device into configuration mode.");
    // Put the device into configuration mode before configuring the device
    if (!m_xsensDevice.gotoConfig()) 
    {
        yError("xsensmt: Could not put device in configuration mode.");
        return false;
    }
    
    // Request the device Id to check the device type
    m_portInfo.setDeviceId(m_xsensDevice.getDeviceId());

    // Check if we have an MTi / MTx / MTmk4 device
    if (!m_portInfo.deviceId().isMt9c() && !m_portInfo.deviceId().isLegacyMtig() && !m_portInfo.deviceId().isMtMk4() && !m_portInfo.deviceId().isFmt_X000())
    {
        yError("xsensmt: No MTi / MTx / MTmk4 device found. Aborting.");
        return false;
    }
    
    yInfo() << "xsensmt: Found a device with id: " << m_portInfo.deviceId().toString().toStdString() << " @ port: " << m_portInfo.portName().toStdString() << ", baudrate: " << m_portInfo.baudrate() << " .";
    yInfo() << "xsensmt: Device: " << m_xsensDevice.getProductCode().toStdString() << " opened.";

    // Configure the device. Note the differences between MTix and MTmk4
    yInfo("xsensmt: Configuring the device of type %s.", m_portInfo.deviceId().toString().c_str());
    if (m_portInfo.deviceId().isMt9c() || m_portInfo.deviceId().isLegacyMtig())
    {
        XsOutputMode outputMode = XOM_Orientation; // output orientation data
        XsOutputSettings outputSettings = XOS_OrientationMode_Quaternion; // output orientation data as quaternion

        yInfo("xsensmt: Device of type %s is not supported by the driver, aborting.", m_portInfo.deviceId().toString().c_str());
     }
     else if (m_portInfo.deviceId().isMtMk4() || m_portInfo.deviceId().isFmt_X000())
     {
        XsOutputConfiguration euler(XDI_EulerAngles, 100);
        XsOutputConfiguration acc(XDI_Acceleration, 100);
        XsOutputConfiguration gyro(XDI_RateOfTurn, 100);
        XsOutputConfiguration mag(XDI_MagneticField, 100);

        XsOutputConfigurationArray configArray;
        configArray.push_back(euler);
        configArray.push_back(acc);
        configArray.push_back(gyro);
        configArray.push_back(mag);
        if (!m_xsensDevice.setOutputConfiguration(configArray))
        {
           yError("xsensmt: Could not configure MTmk4 device. Aborting.");
           return false;
        }
     }
     else
     {
        yError("xsensmt: Unknown device while configuring. Aborting.");
        return false;
     }

     // Put the device in measurement mode
     yInfo() << "xsensmt: Putting device into measurement mode.";
     if (!m_xsensDevice.gotoMeasurement())
     {
        yError("xsensmt: Could not put device into measurement mode. Aborting.");
        return false;
     }
     
    // Initialize the sensor in timeout mode 
    m_isSensorMeasurementAvailable = false;
    m_lastStamp.update(yarp::os::SystemClock::nowSystem()-2*m_timeoutInSecond);
    m_lastReadStamp = m_lastStamp;
                        
    // start thread
    m_sensorThread = std::thread(std::bind(&XsensMT::sensorReadLoop, this));
}

bool XsensMT::close()
{
    m_isDeviceClosing = true;
    if (m_sensorThread.joinable())
    {
        m_sensorThread.join();
    }
}

yarp::os::Stamp XsensMT::getLastInputStamp()
{
    return m_lastReadStamp;
}


void XsensMT::sensorReadLoop()
{
    XsByteArray data;
    XsMessageArray msgs;
 
    XsEuler euler;
    XsVector acc;
    XsVector gyro;
    XsVector mag;
    
    while (!m_isDeviceClosing)
    {
        m_xsensDevice.readDataToBuffer(data);
        m_xsensDevice.processBufferedData(data, msgs);
        for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
        {
            // Retrieve a packet
            XsDataPacket packet;
            if ((*it).getMessageId() == XMID_MtData) {
                LegacyDataPacket lpacket(1, false);
                lpacket.setMessage((*it));
                lpacket.setXbusSystem(false);
                lpacket.setDeviceId(m_portInfo.deviceId(), 0);
                lpacket.setDataFormat(XOM_Orientation, XOS_OrientationMode_Quaternion,0);
                yError("xsensmt: Legacy packet received, but the legacy packet is not currently supported by the driver. Ignoring message.");
                continue;
                //XsDataPacket_assignFromLegacyDataPacket(&packet, &lpacket, 0);
            }
            else if ((*it).getMessageId() == XMID_MtData2) {
                packet.setMessage((*it));
                packet.setDeviceId(m_portInfo.deviceId());
            }
            
            // Get the euler data (for compatibility with the old xsensmtx icub-main driver)
            euler = packet.orientationEuler();
            acc = packet.calibratedAcceleration();
            gyro = packet.calibratedGyroscopeData();
            mag = packet.calibratedMagneticField();
                        
            m_bufferMutex.lock();
            
            // Euler angles are expressed in degree in YARP, 
            m_sensorBuffer[0]  = euler.roll(); //roll
            m_sensorBuffer[1]  = euler.pitch(); //pitch
            m_sensorBuffer[2]  = euler.yaw(); //yaw

            // Accelerometers are expressed in m/s^2 in both YARP and in the Xsens interface
            if (acc.size() == 3) {
                m_sensorBuffer[3]  = acc[0]; //accel-X
                m_sensorBuffer[4]  = acc[1]; //accel-Y
                m_sensorBuffer[5]  = acc[2]; //accel-Z
            } else {
              yWarning() << "xsensmt: Malformed accelerometer message, skipping accelerometer update.";
            }
            
            // Gyroscope data are expressed in rad/s in the Xsens interface, so they have to be converted in deg/s
            if (gyro.size() == 3) {
                m_sensorBuffer[6]  = gyro[0]*CTRL_RAD2DEG;  //gyro-X
                m_sensorBuffer[7]  = gyro[1]*CTRL_RAD2DEG;  //gyro-Y
                m_sensorBuffer[8]  = gyro[2]*CTRL_RAD2DEG;  //gyro-Z
            } else {
                yWarning() << "xsensmt: Malformed gyroscope message, skipping gyroscope update.";
            }
            
            // Magnetometers 
            if (mag.size() == 3) {
                m_sensorBuffer[9]  = mag[0];  //magn-X
                m_sensorBuffer[10] = mag[1];  //magn-Y
                m_sensorBuffer[11] = mag[2];  //magn-Z
            } else {
                yWarning() << "xsensmt: Malformed magnetometer message, skipping magnetometer update.";
            }

            // TODO(traversaro): update sensor available logic
            m_isSensorMeasurementAvailable = true;
            m_lastStamp.update(yarp::os::SystemClock::nowSystem());
            m_bufferMutex.unlock();
        }
        msgs.clear();
        // The Xsens API does not support a blocking read, so this delay 
        // is necessary to avoid busy waiting, but influence the latency 
        // of when a measurement is available in the YARP interface 
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
}

