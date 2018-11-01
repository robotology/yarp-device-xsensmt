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
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>

#include <cstdint>
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
* | sensor_name    | string  |       | xsensmt       | No        | Name of the inertial sensor device. | |
* | frame_name     | string  |       | sensor_imu_xsensmt| No    | Sensor frame in which the measurements are expressed. |
**/
class yarp::dev::XsensMT : public yarp::dev::IGenericSensor,
                           public yarp::dev::IPreciselyTimed,
                           public yarp::dev::DeviceDriver,
                           public yarp::dev::IThreeAxisGyroscopes,
                           public yarp::dev::IThreeAxisLinearAccelerometers,
                           public yarp::dev::IThreeAxisMagnetometers,
                           public yarp::dev::IOrientationSensors
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

    /* IThreeAxisGyroscopes methods */
    virtual size_t getNrOfThreeAxisGyroscopes() const override;
    virtual yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t sens_index) const override;
    virtual bool getThreeAxisGyroscopeName(size_t sens_index, std::string &name) const override;
    virtual bool getThreeAxisGyroscopeFrameName(size_t sens_index, std::string &frameName) const override;
    virtual bool getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;
    
    /* IThreeAxisLinearAccelerometers methods */
    virtual size_t getNrOfThreeAxisLinearAccelerometers() const override;
    virtual yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t sens_index) const override;
    virtual bool getThreeAxisLinearAccelerometerName(size_t sens_index, std::string &name) const override;
    virtual bool getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string &frameName) const override;
    virtual bool getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;
    
    /* IThreeAxisMagnetometers methods */
    virtual size_t getNrOfThreeAxisMagnetometers() const override;
    virtual yarp::dev::MAS_status getThreeAxisMagnetometerStatus(size_t sens_index) const override;
    virtual bool getThreeAxisMagnetometerName(size_t sens_index, std::string &name) const override;
    virtual bool getThreeAxisMagnetometerFrameName(size_t sens_index, std::string &frameName) const override;
    virtual bool getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;
    
    /* IOrientationSensors methods */
    virtual size_t getNrOfOrientationSensors() const override;
    virtual yarp::dev::MAS_status getOrientationSensorStatus(size_t sens_index) const override;
    virtual bool getOrientationSensorName(size_t sens_index, std::string &name) const override;
    virtual bool getOrientationSensorFrameName(size_t sens_index, std::string &frameName) const override;
    virtual bool getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const override;
    
    // Function executed by m_sensorThread
    void sensorReadLoop();

    virtual yarp::os::Stamp getLastInputStamp();

    // Thread running the sensorReadLoop method
    std::thread m_sensorThread;

private:
    yarp::dev::MAS_status genericGetStatus(size_t sens_index) const;
    bool genericGetSensorName(size_t sens_index, std::string &name) const;
    bool genericGetFrameName(size_t sens_index, std::string &frameName) const;
    
    std::string m_sensorName;
    std::string m_frameName;
    
    // Send a SetFilterProfile message to set the filter profile
    bool setFilterProfile(const uint16_t profile);

    // Send a SetOptionFlags message to set the option flags
    bool setOptionFlags(const uint32_t setFlags, const uint32_t clearFlags);

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
    mutable std::mutex m_bufferMutex;
    yarp::sig::Vector m_sensorBuffer;
    yarp::os::Stamp  m_lastStamp;
};


#endif
