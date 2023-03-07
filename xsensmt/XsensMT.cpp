/*
 * Copyright (C) 2017 iCub Facility,  Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */


#include <yarp/os/Log.h>
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

#include "XsensMT.h"

Journaller* gJournal;

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

    if (config.check("xsensmt_period") && ( config.find("xsensmt_period").isInt32() || config.find("xsensmt_period").isFloat64()))
    {
        m_outputPeriod = config.find("xsensmt_period").asFloat64();
    }
    else
    {
        m_outputPeriod = 0.01; // 10ms
        yWarning() << "xsensmt -  Parameter \"xsensmt_period\" not set. Using the default value " << m_outputPeriod << " seconds for this parameter.";
    }
    m_outputFrequency = 1/m_outputPeriod;

    std::string comPortString = config.check("serial", yarp::os::Value("/dev/ttyUSB0"), "File of the serial device.").asString().c_str();
    int baudRate = config.check("baud", yarp::os::Value(115200), "Baud rate used by the serial communication.").asInt32();
    m_timeoutInSecond = config.check("timeout", yarp::os::Value(0.1), "Timeout of the driver").asFloat64();

    m_portInfo = XsPortInfo(comPortString, XsBaud::numericToRate(baudRate));

    m_xsensControl = XsControl::construct();
    assert(m_xsensControl != 0);


    yInfo("xsensmt: Opening serial port %s with baud rate %d and output period %4.4f seconds.", comPortString.c_str(), baudRate, m_outputPeriod);
    if (!m_xsensControl->openPort(comPortString, XsBaud::numericToRate(baudRate)))
    {
        yError("Could not open port. Aborting.");
        return false;
    }

    // Get the device object
    m_xsensDevice = m_xsensControl->device(m_portInfo.deviceId());
    assert(m_xsensDevice != 0);

    yDebug() << "Device: " << m_xsensDevice->productCode().toStdString() << ", with ID: " << m_xsensDevice->deviceId().toString().toStdString() << " opened.";

    yInfo("xsensmt: Putting device into configuration mode.");
    // Put the device into configuration mode before configuring the device
    if (!m_xsensDevice->gotoConfig())
    {
        yError("xsensmt: Could not put device in configuration mode.");
        return false;
    }

    // Request the device Id to check the device type
    m_portInfo.setDeviceId(m_xsensDevice->deviceId());

    // Check if we have an MTi
    if (!m_portInfo.deviceId().isMti())
    {
        yError("xsensmt: No MTi device found. Aborting.");
        return false;
    }

    yInfo() << "xsensmt: Found a device with id: " << m_portInfo.deviceId().toString().toStdString() << " @ port: " << m_portInfo.portName().toStdString() << ", baudrate: " << m_portInfo.baudrate() << " .";
    // TODO migrate to modern API
    yInfo() << "xsensmt: Device: " << m_portInfo.deviceId().productCode().toStdString() << " opened.";

    // Configure the device. First Check if the device is an MTi
    yInfo("xsensmt: Configuring the device of type %s.", m_portInfo.deviceId().toString().c_str());
    if (!m_portInfo.deviceId().isMti())
    {
        yError("xsensmt: Device of type %s is not supported by the driver, aborting.", m_portInfo.deviceId().toString().c_str());
        return false;
    }
    else
    {
        XsOutputConfigurationArray configArray;

        if (m_xsensDevice->deviceId().isImu())
        {
            XsOutputConfiguration acc(XDI_Acceleration, m_outputFrequency);
            XsOutputConfiguration gyro(XDI_RateOfTurn, m_outputFrequency);
            XsOutputConfiguration mag(XDI_MagneticField, m_outputFrequency);

            configArray.push_back(acc);
            configArray.push_back(gyro);
            configArray.push_back(mag);

            if (!m_xsensDevice->setOutputConfiguration(configArray))
            {
               yError("xsensmt: Could not configure device. Aborting.");
               return false;
            }
        }
        else if (m_xsensDevice->deviceId().isVru() || m_xsensDevice->deviceId().isAhrs())
        {
            XsOutputConfiguration euler(XDI_EulerAngles, m_outputFrequency);
            // IMU stuff
            XsOutputConfiguration acc(XDI_Acceleration, m_outputFrequency);
            XsOutputConfiguration gyro(XDI_RateOfTurn, m_outputFrequency);
            XsOutputConfiguration mag(XDI_MagneticField, m_outputFrequency);

            configArray.push_back(euler);
            configArray.push_back(acc);
            configArray.push_back(gyro);
            configArray.push_back(mag);

            if (!m_xsensDevice->setOutputConfiguration(configArray))
            {
               yError("xsensmt: Could not configure device. Aborting.");
               return false;
            }
        }
        else if (m_xsensDevice->deviceId().isGnss())
        {
            XsOutputConfiguration euler(XDI_EulerAngles, m_outputFrequency);
            XsOutputConfiguration posHorizontal(XDI_LatLon, m_outputFrequency);
            XsOutputConfiguration posVertical(XDI_AltitudeEllipsoid, m_outputFrequency);
            XsOutputConfiguration linVel(XDI_VelocityXYZ, m_outputFrequency);
            // IMU stuff
            XsOutputConfiguration acc(XDI_Acceleration, m_outputFrequency);
            XsOutputConfiguration gyro(XDI_RateOfTurn, m_outputFrequency);
            XsOutputConfiguration mag(XDI_MagneticField, m_outputFrequency);

            configArray.push_back(euler);
            configArray.push_back(posHorizontal);
            configArray.push_back(posVertical);
            configArray.push_back(linVel);
            configArray.push_back(acc);
            configArray.push_back(gyro);
            configArray.push_back(mag);

            if (!m_xsensDevice->setOutputConfiguration(configArray))
            {
               yError("xsensmt: Could not configure device. Aborting.");
               return false;
            }
        }
        else
        {
            yError("xsensmt: Unknown device while configuring. Aborting.");
            return false;
        }
    }

    // Put the device in measurement mode
    yInfo() << "xsensmt: Putting device into measurement mode.";
    if (!m_xsensDevice->gotoMeasurement())
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

    // Create and attach callback handler to device
    m_xsensDevice->addCallbackHandler(&m_callback);

    return true;
}

bool XsensMT::close()
{
    m_isDeviceClosing = true;
    if (m_sensorThread.joinable())
    {
        m_sensorThread.join();
    }

    if(m_xsensControl)
    {
        yDebug("xsensmt: Closing Xsens port %s.", m_portInfo.portName().toStdString().c_str());
        m_xsensControl->closePort(m_portInfo.portName().toStdString());

        yDebug("xsensmt: Freeing Xsens control object.");
        m_xsensControl->destruct();
        m_xsensControl = nullptr;
    }

        return true;
}

yarp::os::Stamp XsensMT::getLastInputStamp()
{
    return m_lastReadStamp;
}


void XsensMT::sensorReadLoop()
{
    XsEuler euler;
    XsVector acc;
    XsVector gyro;
    XsVector mag;

    while (!m_isDeviceClosing)
    {
        if (m_callback.packetAvailable())
        {
            // Retrieve a packet
            XsDataPacket packet = m_callback.getNextPacket();

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
        // The Xsens API does not support a blocking read, so this delay
        // is necessary to avoid busy waiting, but influence the latency
        // of when a measurement is available in the YARP interface
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }
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
