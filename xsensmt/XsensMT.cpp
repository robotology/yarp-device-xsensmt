/*
 * Copyright (C) 2017 iCub Facility,  Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include <yarp/os/LogStream.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Stamp.h>

#include <chrono>
#include <iostream>
#include <string>
#include <functional>

#include <xcommunication/int_xsdatapacket.h>
#include <xcommunication/legacydatapacket.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xsmessagearray.h>

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
    if (config.check("sensor_name") && config.find("sensor_name").isString())
    {
        m_sensorName = config.find("sensor_name").asString();
    }
    else
    {
        m_sensorName = "sensor_imu_xsensmt";
        yWarning() << "xsensmt -  Parameter \"sensor_name\" not set. Using default value  \"" << m_sensorName << "\" for this parameter.";
    }
    
    if (config.check("frame_name") && config.find("frame_name").isString())
    {
        m_frameName = config.find("frame_name").asString();
    }
    else
    {
        m_frameName = m_sensorName;
        yWarning() << "xsensmt -  Parameter \"frame_name\" not set. Using the same value as \"sensor_name\" for this parameter.";
    }

    yDebug() << "xsensmt - test - xsensmt_period, check xsensmt_period: " << config.check("xsensmt_period");
    yDebug() << "xsensmt - test - xsensmt_period, xsensmt_period is string: " << config.find("xsensmt_period").isString();
    yDebug() << "xsensmt - test - xsensmt_period, xsensmt_period is int: " << config.find("xsensmt_period").isInt32();
    yDebug() << "xsensmt - test - xsensmt_period, xsensmt_period is float: " << config.find("xsensmt_period").isFloat64();

    if (config.check("xsensmt_period") && config.find("xsensmt_period").isString())
    {
        m_outputPeriod = config.find("xsensmt_period").asFloat64();
    }
    else
    {
        m_outputPeriod = 10; // 10ms
        yWarning() << "xsensmt -  Parameter \"xsensmt_period\" not set. Using the value " << m_outputPeriod << " ms for this parameter.";
    }
    m_outputFrequency = 1/m_outputPeriod * 1000;

    std::string comPortString = config.check("serial", yarp::os::Value("/dev/ttyUSB0"), "File of the serial device.").asString().c_str();
    int baudRate = config.check("baud", yarp::os::Value(115200), "Baud rate used by the serial communication.").asInt32();
    m_timeoutInSecond = config.check("timeout", yarp::os::Value(0.1), "Timeout of the driver").asFloat64();

    m_portInfo = XsPortInfo(comPortString, XsBaud::numericToRate(baudRate));

    yInfo("xsensmt: Opening serial port %s with baud rate %d and output period %4.2f ms.", comPortString.c_str(), baudRate, m_outputPeriod);
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
        yError("xsensmt: Device of type %s is not supported by the driver, aborting.", m_portInfo.deviceId().toString().c_str());
        return false;
     }
     else if (m_portInfo.deviceId().isMtMk4() || m_portInfo.deviceId().isFmt_X000())
     {
        XsOutputConfiguration euler(XDI_EulerAngles, m_outputFrequency);
        XsOutputConfiguration acc(XDI_Acceleration, m_outputFrequency);
        XsOutputConfiguration gyro(XDI_RateOfTurn, m_outputFrequency);
        XsOutputConfiguration mag(XDI_MagneticField, m_outputFrequency);

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

        // Paramater configuration

        /// Profile configuration
        // Possible profiles
        // Profile numbers extracted from Document MT0101P.2018.B - MT Low Level Communication Documentation
        // Documentation of setFilterProfile message, page 48
        // https://xsens.com/download/usermanual/ISM/MT_LowLevelCommunicationProtocol_Documentation.pdf
        uint16_t VRU_general = 43;
        if (!this->setFilterProfile(VRU_general))
        {
           yError("xsensmt: Failing in sending SetFilterProfile message. Aborting.");
           return false;
        }

        /// Options configuration
        // Flags numbers extracted from Document MT0101P.2018.B - MT Low Level Communication Documentation
        // Documentation of SetOptionFlags message, page 17
        // https://xsens.com/download/usermanual/ISM/MT_LowLevelCommunicationProtocol_Documentation.pdf
        uint32_t EnableAhs = 0x00000010;
        uint32_t SetFlags = EnableAhs;
        uint32_t ClearFlags = 0x0;
        if (!this->setOptionFlags(SetFlags, ClearFlags))
        {
           yError("xsensmt: Failing in sending SetOptionFlags message. Aborting.");
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

    return true;
}

bool XsensMT::close()
{
    m_isDeviceClosing = true;
    if (m_sensorThread.joinable())
    {
        m_sensorThread.join();
    }
    return true;
}

yarp::os::Stamp XsensMT::getLastInputStamp()
{
    return m_lastReadStamp;
}


void XsensMT::sensorReadLoop()
{
    XsByteArray data;
    std::deque<XsMessage> msgs;

    XsEuler euler;
    XsVector acc;
    XsVector gyro;
    XsVector mag;

    while (!m_isDeviceClosing)
    {
        m_xsensDevice.readDataToBuffer(data);
        m_xsensDevice.processBufferedData(data, msgs);
        for (std::deque<XsMessage>::iterator it = msgs.begin(); it != msgs.end(); ++it)
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
            if (packet.containsOrientation()) {
                m_sensorBuffer[0]  = euler.roll(); //roll
                m_sensorBuffer[1]  = euler.pitch(); //pitch
                m_sensorBuffer[2]  = euler.yaw(); //yaw
            } else {
              yWarning() << "xsensmt: Missing orientation message, skipping orientation update.";
            }

            // Accelerometers are expressed in m/s^2 in both YARP and in the Xsens interface
            if (packet.containsCalibratedAcceleration()) {
                m_sensorBuffer[3]  = acc[0]; //accel-X
                m_sensorBuffer[4]  = acc[1]; //accel-Y
                m_sensorBuffer[5]  = acc[2]; //accel-Z
            } else {
              yWarning() << "xsensmt: Missing accelerometer message, skipping accelerometer update.";
            }

            // Gyroscope data are expressed in rad/s in the Xsens interface, so they have to be converted in deg/s
            if (packet.containsCalibratedGyroscopeData()) {
                m_sensorBuffer[6]  = gyro[0]*CTRL_RAD2DEG;  //gyro-X
                m_sensorBuffer[7]  = gyro[1]*CTRL_RAD2DEG;  //gyro-Y
                m_sensorBuffer[8]  = gyro[2]*CTRL_RAD2DEG;  //gyro-Z
            } else {
                yWarning() << "xsensmt: Missing gyroscope message, skipping gyroscope update.";
            }

            // Magnetometers
            if (packet.containsCalibratedMagneticField()) {
                m_sensorBuffer[9]  = mag[0];  //magn-X
                m_sensorBuffer[10] = mag[1];  //magn-Y
                m_sensorBuffer[11] = mag[2];  //magn-Z
            } else {
                yWarning() << "xsensmt: Missing magnetometer message, skipping magnetometer update.";
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

bool XsensMT::setFilterProfile(const uint16_t profile)
{
    XsMessage snd(XMID_SetFilterProfile, 2), rcv;
    snd.setDataShort(profile, 0);
    m_xsensDevice.writeMessage(snd);
    return m_xsensDevice.waitForMessage(XMID_SetFilterProfileAck, rcv);
}

bool XsensMT::setOptionFlags(const uint32_t setFlags, const uint32_t clearFlags)
{
    XsMessage snd(XMID_SetOptionFlags, 8), rcv;
    // SetFlags (4 Bytes)
    snd.setDataLong(setFlags, 0);
    // ClearFlags (4 bytes)
    snd.setDataLong(clearFlags, 4);
    m_xsensDevice.writeMessage(snd);
    return m_xsensDevice.waitForMessage(XMID_SetOptionFlagsAck, rcv);
}

yarp::dev::MAS_status XsensMT::genericGetStatus(size_t sens_index) const
{
    if (sens_index != 0)
    {
        yError() << "xsensmt: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return yarp::dev::MAS_status::MAS_ERROR;
    }
    
    return yarp::dev::MAS_status::MAS_OK;
}

bool XsensMT::genericGetSensorName(size_t sens_index, std::string& name) const
{
    if (sens_index != 0)
    {
        yError() << "xsensmt: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }
    
    name = m_sensorName;
    return true;
}

bool XsensMT::genericGetFrameName(size_t sens_index, std::string& frameName) const
{
    if (sens_index != 0)
    {
        yError() << "xsensmt: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }
    
    frameName = m_frameName;
    return true;
}

size_t XsensMT::getNrOfOrientationSensors() const
{
    return 1;
}

yarp::dev::MAS_status XsensMT::getOrientationSensorStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool XsensMT::getOrientationSensorName(size_t sens_index, std::string& name) const
{
    return genericGetSensorName(sens_index, name);
}


bool XsensMT::getOrientationSensorFrameName(size_t sens_index, std::string& frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool XsensMT::getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const
{
    if (sens_index != 0 || m_isSensorMeasurementAvailable == false)
    {
        yError() << "xsensmt: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }
    
    rpy.resize(3);    
    std::lock_guard<std::mutex> guard(m_bufferMutex);
    rpy[0] = m_sensorBuffer[0];
    rpy[1] = m_sensorBuffer[1];
    rpy[2] = m_sensorBuffer[2];
    yarp::os::Stamp copyStamp(m_lastStamp);
    timestamp = copyStamp.getTime();
    return true;
}


size_t XsensMT::getNrOfThreeAxisLinearAccelerometers() const
{
    return 1;
}

yarp::dev::MAS_status XsensMT::getThreeAxisLinearAccelerometerStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool XsensMT::getThreeAxisLinearAccelerometerName(size_t sens_index, std::string& name) const
{
    return genericGetSensorName(sens_index, name);
}

bool XsensMT::getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string& frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool XsensMT::getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (sens_index != 0 || m_isSensorMeasurementAvailable == false)
    {
        yError() << "xsensmt: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }
    
    out.resize(3);
    std::lock_guard<std::mutex> guard(m_bufferMutex);
    out[0] = m_sensorBuffer[3];
    out[1] = m_sensorBuffer[4];
    out[2] = m_sensorBuffer[5];
    yarp::os::Stamp copyStamp(m_lastStamp);
    timestamp = copyStamp.getTime();
    return true;
}


size_t XsensMT::getNrOfThreeAxisGyroscopes() const
{
    return 1;
}

yarp::dev::MAS_status XsensMT::getThreeAxisGyroscopeStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool XsensMT::getThreeAxisGyroscopeName(size_t sens_index, std::string& name) const
{
    return genericGetSensorName(sens_index, name);
}


bool XsensMT::getThreeAxisGyroscopeFrameName(size_t sens_index, std::string& frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool XsensMT::getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (sens_index != 0 || m_isSensorMeasurementAvailable == false)
    {
        yError() << "xsensmt: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }
    
    out.resize(3);    
    std::lock_guard<std::mutex> guard(m_bufferMutex);
    out[0] = m_sensorBuffer[6];
    out[1] = m_sensorBuffer[7];
    out[2] = m_sensorBuffer[8];
    yarp::os::Stamp copyStamp(m_lastStamp);
    timestamp = copyStamp.getTime();
    return true;
}



size_t XsensMT::getNrOfThreeAxisMagnetometers() const
{
    return 1;
}


yarp::dev::MAS_status XsensMT::getThreeAxisMagnetometerStatus(size_t sens_index) const
{
    return genericGetStatus(sens_index);
}

bool XsensMT::getThreeAxisMagnetometerName(size_t sens_index, std::string& name) const
{
    return genericGetSensorName(sens_index, name);
}

bool XsensMT::getThreeAxisMagnetometerFrameName(size_t sens_index, std::string& frameName) const
{
    return genericGetFrameName(sens_index, frameName);
}

bool XsensMT::getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (sens_index != 0 || m_isSensorMeasurementAvailable == false)
    {
        yError() << "xsensmt: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }
    
    out.resize(3);
    std::lock_guard<std::mutex> guard(m_bufferMutex);
    out[0] = m_sensorBuffer[9];
    out[1] = m_sensorBuffer[10];
    out[2] = m_sensorBuffer[11];
    yarp::os::Stamp copyStamp(m_lastStamp);
    timestamp = copyStamp.getTime();
    return true;
}
