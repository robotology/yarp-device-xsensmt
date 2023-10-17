// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause


#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#include <chrono>
#include <iostream>
#include <mutex>
#include <string>
#include <functional>

#include "XsensMT.h"
#include "xsbaud.h"

Journaller* gJournal;

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

#define XSENS_PI             3.14159265358979323846264338328
#define CTRL_RAD2DEG         (180.0/XSENS_PI)
#define CTRL_DEG2RAD         (XSENS_PI/180.0)

TimedData::TimedData()
{
    m_data.resize(3);
}

void TimedData::setData(const XsEuler& euler,
                        const yarp::os::Stamp& time,
                        std::function<double(const double&)> conversion)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_timestamp = time;
    m_dataArrived = true;

    m_data[0] = conversion(euler.roll());
    m_data[1] = conversion(euler.pitch());
    m_data[2] = conversion(euler.yaw());
}

void TimedData::setData(const XsVector& vector,
                        const yarp::os::Stamp& time,
                        std::function<double(const double&)> conversion)
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_timestamp = time;
    m_dataArrived = true;

    // if the size is different resize
    if (m_data.size() != vector.size())
    {
        m_data.resize(vector.size());
    }

    std::transform(vector.data(),
                   vector.data() + vector.size(),
                   m_data.data(),
                   conversion);
}

bool TimedData::getData(yarp::sig::Vector& data, double& timestamp) const
{
    std::lock_guard<std::mutex> guard(m_mutex);
    if(!m_dataArrived)
    {
        return false;
    }

    data = m_data;
    timestamp = m_timestamp.getTime();

    return true;
}

void SensorDataCollection::updateLastReadStamp(const double& t)
{
    std::lock_guard<std::mutex> guard(m_lastReadStampMutex);
    m_lastReadStamp.update(t);
}

double SensorDataCollection::getLastReadStampAsDouble()
{
    std::lock_guard<std::mutex> guard(m_lastReadStampMutex);
    return m_lastReadStamp.getTime();
}

yarp::os::Stamp SensorDataCollection::getLastReadStamp()
{
   std::lock_guard<std::mutex> guard(m_lastReadStampMutex);
   return m_lastReadStamp;
}

void CallbackHandler::onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
{
    assert(packet != 0);

    XsEuler euler;
    XsVector acc;
    XsVector gyro;
    XsVector mag;

    // Get the euler data (for compatibility with the old xsensmtx icub-main driver)
    euler = packet->orientationEuler();
    acc = packet->calibratedAcceleration();
    gyro = packet->calibratedGyroscopeData();
    mag = packet->calibratedMagneticField();

    // at least one signal is arrived
    this->packetBuffer.updateLastReadStamp(yarp::os::SystemClock::nowSystem());
    const yarp::os::Stamp timestamp = this->packetBuffer.getLastReadStamp();

    // Euler angles are expressed in degree in YARP,
    if (packet->containsOrientation()) {
        this->packetBuffer.euler.setData(euler, timestamp);
    }

    // Accelerometers are expressed in m/s^2 in both YARP and in the Xsens interface
    if (packet->containsCalibratedAcceleration()) {
        this->packetBuffer.acc.setData(acc, timestamp);
    }

    // Gyroscope data are expressed in rad/s in the Xsens interface, so they have to be converted in deg/s
    if (packet->containsCalibratedGyroscopeData()) {
        this->packetBuffer.gyro.setData(gyro, timestamp,
                                        [](const double& rad){return rad * CTRL_RAD2DEG;});
    }

    // Magnetometers
    if (packet->containsCalibratedMagneticField()) {
        this->packetBuffer.mag.setData(mag, timestamp);
    }
}

XsensMT::XsensMT()
{
}

XsensMT::~XsensMT()
{
    close();
}

bool XsensMT::read(Vector &out)
{
    if(out.size() != m_nchannels)
    {
        out.resize(m_nchannels);
    }

    yarp::sig::Vector temp(3);
    double tempTimeStamp;
    auto fillVector = [&temp, &tempTimeStamp, &out](TimedData& timedData, std::size_t indexOffset) -> bool {
                          if(timedData.getData(temp, tempTimeStamp))
                          {
                              out[0 + indexOffset] = temp[0];
                              out[1 + indexOffset] = temp[1];
                              out[2 + indexOffset] = temp[2];
                              return true;
                          }
                          return false;
                      };

    bool ret = fillVector(m_callback.packetBuffer.euler, 0);

    // the order matter. In the other case we will suffer the short circuit rule of the Boolean
    // operators
    ret = fillVector(m_callback.packetBuffer.acc, 3) || ret;
    ret = fillVector(m_callback.packetBuffer.gyro, 6) || ret;
    ret = fillVector(m_callback.packetBuffer.mag, 9) || ret;

    const double now = yarp::os::SystemClock::nowSystem();
    const double latestRead = m_callback.packetBuffer.getLastReadStampAsDouble();
    if (now-latestRead > m_timeoutInSecond)
    {
        yError("xsensmt: Sensor timeout, no sensor measurement received in the last %lf seconds.",
               now-latestRead);
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
    const char* logPrefix = "xsensmt - ";
    auto getStringFromParam = [logPrefix, &config] (const std::string& parameterName, const std::string& defaultParameter) -> std::string
                              {
                                  if (config.check(parameterName) && config.find(parameterName).isString())
                                  {
                                      return config.find(parameterName).asString();
                                  }

                                  yWarning() << logPrefix << "Parameter '" << parameterName
                                             << "' not set. Using default value '"
                                             << defaultParameter << "' for this parameter.";
                                  return defaultParameter;
                              };

    auto getDoubleFromParam = [logPrefix, &config] (const std::string& parameterName, const double& defaultParameter = -1, bool verbose = false) -> double
                              {
                                  if (config.check(parameterName) &&
                                      (config.find(parameterName).isInt32() || config.find(parameterName).isFloat64()))
                                  {
                                      return config.find(parameterName).asFloat64();
                                  }
                                  if (verbose)
                                  {
                                      yWarning() << logPrefix << "Parameter '" << parameterName
                                                 << "' not set. Using default value '"
                                                 << defaultParameter << "' for this parameter.";
                                  }
                                  return defaultParameter;
                              };

    m_sensorName = getStringFromParam("sensor_name", "sensor_imu_xsensmt");
    m_frameName = getStringFromParam("frame_name", m_sensorName);

    const double generalPeriod =  getDoubleFromParam("xsensmt_period", 0.01, true);
    m_frequencies.acc = 1. / getDoubleFromParam("xsensmt_acc_period", generalPeriod);
    m_frequencies.gyro = 1. / getDoubleFromParam("xsensmt_gyro_period", generalPeriod);
    m_frequencies.mag = 1. / getDoubleFromParam("xsensmt_mag_period", generalPeriod);
    m_frequencies.euler = 1. / getDoubleFromParam("xsensmt_euler_period", generalPeriod);
    m_frequencies.position = 1. / getDoubleFromParam("xsensmt_position_period", generalPeriod);
    m_frequencies.linearVelocity = 1. / getDoubleFromParam("xsensmt_linear_velocity_period", generalPeriod);

    std::string comPortString = config.check("serial", yarp::os::Value("/dev/ttyUSB0"), "File of the serial device.").asString().c_str();
    int baudRate = config.check("baud", yarp::os::Value(115200), "Baud rate used by the serial communication.").asInt32();
    m_timeoutInSecond = config.check("timeout", yarp::os::Value(0.1), "Timeout of the driver").asFloat64();

    m_portInfo = XsPortInfo(comPortString, XsBaud::numericToRate(baudRate));

    m_xsensControl = XsControl::construct();
    assert(m_xsensControl != 0);

    yInfo("xsensmt: Opening serial port %s with baud rate %d. The following frequencies are considered. "
          "Accelerometer %4.4f Hz, gyro %4.4f Hz, magnetometer %4.4f Hz, euler angle %4.4f Hz, position %4.4f Hz, linearVelocity %4.4f Hz",
          comPortString.c_str(), baudRate, m_frequencies.acc, m_frequencies.gyro, m_frequencies.mag, m_frequencies.euler, m_frequencies.position, m_frequencies.linearVelocity);

    if (!m_xsensControl->openPort(comPortString, XsBaud::numericToRate(baudRate)))
    {
        yError("Could not open port. Aborting.");
        return false;
    }

    // Get the device object
    m_xsensDevice = m_xsensControl->device(m_portInfo.deviceId());
    assert(m_xsensDevice != 0);

    yInfo("Device: %s, with ID: %s opened.", m_xsensDevice->productCode().toStdString().c_str(), m_xsensDevice->deviceId().toString().toStdString().c_str());
    // << m_xsensDevice->productCode().toStdString() << ", with ID: " << m_xsensDevice->deviceId().toString().toStdString() << " opened.";

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

    // Configure the device. First Check if the device is an MTi
    yInfo("xsensmt: Configuring the device %s.", m_xsensDevice->productCode().toStdString().c_str());
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
            XsOutputConfiguration acc(XDI_Acceleration, m_frequencies.acc);
            XsOutputConfiguration gyro(XDI_RateOfTurn, m_frequencies.gyro);
            XsOutputConfiguration mag(XDI_MagneticField, m_frequencies.mag);

            configArray.push_back(acc);
            configArray.push_back(gyro);
            configArray.push_back(mag);
        }
        else if (m_xsensDevice->deviceId().isVru() || m_xsensDevice->deviceId().isAhrs())
        {
            XsOutputConfiguration euler(XDI_EulerAngles, m_frequencies.euler);
            // IMU stuff
            XsOutputConfiguration acc(XDI_Acceleration, m_frequencies.acc);
            XsOutputConfiguration gyro(XDI_RateOfTurn, m_frequencies.gyro);
            XsOutputConfiguration mag(XDI_MagneticField, m_frequencies.mag);

            configArray.push_back(euler);
            configArray.push_back(acc);
            configArray.push_back(gyro);
            configArray.push_back(mag);
        }
        else if (m_xsensDevice->deviceId().isGnss())
        {
            XsOutputConfiguration euler(XDI_EulerAngles, m_frequencies.euler);
            XsOutputConfiguration posHorizontal(XDI_LatLon, m_frequencies.position);
            XsOutputConfiguration posVertical(XDI_AltitudeEllipsoid, m_frequencies.position);
            XsOutputConfiguration linVel(XDI_VelocityXYZ, m_frequencies.linearVelocity);
            // IMU stuff
            XsOutputConfiguration acc(XDI_Acceleration, m_frequencies.acc);
            XsOutputConfiguration gyro(XDI_RateOfTurn, m_frequencies.gyro);
            XsOutputConfiguration mag(XDI_MagneticField, m_frequencies.mag);

            configArray.push_back(euler);
            configArray.push_back(posHorizontal);
            configArray.push_back(posVertical);
            configArray.push_back(linVel);
            configArray.push_back(acc);
            configArray.push_back(gyro);
            configArray.push_back(mag);
        }
        else
        {
            yError("xsensmt: Unknown device while configuring. Aborting.");
            return false;
        }

        if (!m_xsensDevice->setOutputConfiguration(configArray))
        {
            yError("xsensmt: Could not configure device. Aborting.");
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

    // Create and attach callback handler to device
    m_xsensDevice->addCallbackHandler(&m_callback);

    return true;
}

bool XsensMT::close()
{
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
    return m_callback.packetBuffer.getLastReadStamp();
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
    if (sens_index != 0)
    {
        yError() << "xsensmt: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }

    rpy.resize(3);

    return m_callback.packetBuffer.euler.getData(rpy, timestamp);
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
    if (sens_index != 0)
    {
        yError() << "xsensmt: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }

    out.resize(3);

    // this is thread safe
    return m_callback.packetBuffer.acc.getData(out, timestamp);
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
    if (sens_index != 0)
    {
        yError() << "xsensmt: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }

    out.resize(3);
    return m_callback.packetBuffer.gyro.getData(out,timestamp);
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
    if (sens_index != 0)
    {
        yError() << "xsensmt: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }

    out.resize(3);
    return m_callback.packetBuffer.mag.getData(out,timestamp);
}
