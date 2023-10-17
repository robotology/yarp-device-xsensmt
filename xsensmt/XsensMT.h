// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef XSENS_MT_YARP_DRIVER
#define XSENS_MT_YARP_DRIVER

#include <xstypes/xsdatapacket.h>
#include <xstypes/xsportinfo.h>
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/mtibasedevice.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IGenericSensor.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/sig/Vector.h>

#include <cstdint>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <list>
#include <functional>

namespace yarp
{
namespace dev
{
    class XsensMT;
}
}

class TimedData
{
    yarp::sig::Vector m_data;
    yarp::os::Stamp m_timestamp;
    bool m_dataArrived{false};
    mutable std::mutex m_mutex;

public:
    TimedData();

    void setData(const XsEuler& euler,
                 const yarp::os::Stamp& time,
                 std::function<double(const double&)> conversion = [](const double& element){return element;});

    void setData(const XsVector& vector,
                 const yarp::os::Stamp& time,
                 std::function<double(const double&)> conversion = [](const double& element){return element;});


    bool getData(yarp::sig::Vector& data, double& timestamp) const;
};

struct SensorDataCollection
{
    TimedData euler;
    TimedData acc;
    TimedData gyro;
    TimedData mag;

    void updateLastReadStamp(const double& t);
    double getLastReadStampAsDouble();
    yarp::os::Stamp getLastReadStamp();

private:
    mutable std::mutex m_lastReadStampMutex;
    yarp::os::Stamp m_lastReadStamp;
};

/**
 * \section CallbackHandler Controls reading data from XsDevice object
 * \brief This class is copied from the Public Xsens device API example MTi receive data.
**/
class CallbackHandler : public XsCallback
{
public:

    virtual ~CallbackHandler() throw() = default;
    SensorDataCollection packetBuffer;

protected:
    void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override;
};


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
* | sensor_name    | string  |       | sensor_imu_xsensmt       | No        | Name of the inertial sensor device. | |
* | frame_name     | string  |       | set same as `sensor_name` | No    | Sensor frame in which the measurements are expressed. |
* | xsensmt_period     | double  |    seconds   | 0.01  | No    | Period of querying the Xsens MT* device. This parameter is used for each sensor for which the user does not specify the specific associated period.   | The frequency of publishing the information is determined by the device that attaches this one |
* | xsensmt_acc_period     | double  |    seconds   | xsensmt_period  | No    | Period of querying the Accelerometer Xsens MT* device | The frequency of publishing the information is determined by the device that attaches this one |
* | xsensmt_gyro_period     | double  |    seconds   | xsensmt_period  | No   | Period of querying the Gyroscope Xsens MT* device | The frequency of publishing the information is determined by the device that attaches this one |
* | xsensmt_mag_period     | double  |    seconds   | xsensmt_period  | No   | Period of querying the Magnetometer Xsens MT* device | The frequency of publishing the information is determined by the device that attaches this one |
* | xsensmt_euler_period     | double  |    seconds   | xsensmt_period  | No   | Period of querying the Euler Xsens MT* device | The frequency of publishing the information is determined by the device that attaches this one |
* | xsensmt_position_period     | double  |    seconds   | xsensmt_period  | No   | Period of querying the Position Xsens MT* device | The frequency of publishing the information is determined by the device that attaches this one |
* | xsensmt_linear_velocity_period     | double  |    seconds   | xsensmt_period  | No   | Period of querying the linear velocity Xsens MT* device | The frequency of publishing the information is determined by the device that attaches this one |
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
    /**
     * Read a vector from the sensor.
     * @param[out] out a vector containing the sensor's last readings.
     * @return true/false success/failure
     */
    virtual bool read(yarp::sig::Vector &out) override;

    /**
     * Get the number of channels of the sensor.
     * @param[out] nc pointer to storage, return value
     * @return true/false success/failure
     */
    virtual bool getChannels(int *nc) override;

    /**
     * Open the device and set up parameters/communication
     * @param[in] config searchable object with desired configuration parameters
     * @return true/false success/failure
     */
    virtual bool open(yarp::os::Searchable &config) override;

    /**
     * Calibrate the sensor, single channel.
     * @param[in] ch channel number
     * @param[in] v reset value
     * @return true/false success/failure
     */
    virtual bool calibrate(int ch, double v) override;

    /**
     * Close the device
     * @return true/false success/failure
     */
    virtual bool close() override;

    /* IThreeAxisGyroscopes methods */
    /**
     * Get the  number of three axis gyroscopes in the device
     * @return 1
     */
    virtual size_t getNrOfThreeAxisGyroscopes() const override;

    /**
     * Get the status of three axis gyroscope
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @return MAS_OK/MAS_ERROR if status ok/failure
     */
    virtual yarp::dev::MAS_status getThreeAxisGyroscopeStatus(size_t sens_index) const override;

    /**
     * Get the name of three axis gyroscope
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @param[out] name name of the sensor
     * @return true/false success/failure
     */
    virtual bool getThreeAxisGyroscopeName(size_t sens_index, std::string &name) const override;

    /**
     * Get the name of the frame in which three axis gyroscope measurements are expressed
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @param[out] frameName name of the sensor frame
     * @return true/false success/failure
     */
    virtual bool getThreeAxisGyroscopeFrameName(size_t sens_index, std::string &frameName) const override;


    /**
     * Get three axis gyroscope measurements
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @param[out] out 3D angular velocity measurement in deg/s
     * @param[out] timestamp timestamp of measurement
     * @return true/false success/failure
     */
    virtual bool getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisLinearAccelerometers methods */
    /**
     * Get the  number of three axis linear accelerometers in the device
     * @return 1
     */
    virtual size_t getNrOfThreeAxisLinearAccelerometers() const override;

    /**
     * Get the status of three axis linear accelerometer
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @return MAS_OK/MAS_ERROR if status ok/failure
     */
    virtual yarp::dev::MAS_status getThreeAxisLinearAccelerometerStatus(size_t sens_index) const override;

    /**
     * Get the name of three axis linear accelerometer
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @param[out] name name of the sensor
     * @return true/false success/failure
     */
    virtual bool getThreeAxisLinearAccelerometerName(size_t sens_index, std::string &name) const override;

    /**
     * Get the name of the frame in which three axis linear accelerometer measurements are expressed
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @param[out] frameName name of the sensor frame
     * @return true/false success/failure
     */
    virtual bool getThreeAxisLinearAccelerometerFrameName(size_t sens_index, std::string &frameName) const override;

    /**
     * Get three axis linear accelerometer measurements
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @param[out] out 3D linear acceleration measurement in m/s^2
     * @param[out] timestamp timestamp of measurement
     * @return true/false success/failure
     */
    virtual bool getThreeAxisLinearAccelerometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IThreeAxisMagnetometers methods */
    /**
     * Get the  number of three axis magnetometers in the device
     * @return 1
     */
    virtual size_t getNrOfThreeAxisMagnetometers() const override;

    /**
     * Get the status of three axis magnetometer
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @return MAS_OK/MAS_ERROR if status ok/failure
     */
    virtual yarp::dev::MAS_status getThreeAxisMagnetometerStatus(size_t sens_index) const override;

    /**
     * Get the name of three axis magnetometer
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @param[out] name name of the sensor
     * @return true/false success/failure
     */
    virtual bool getThreeAxisMagnetometerName(size_t sens_index, std::string &name) const override;

    /**
     * Get the name of the frame in which three axis magnetometer measurements are expressed
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @param[out] frameName name of the sensor frame
     * @return true/false success/failure
     */
    virtual bool getThreeAxisMagnetometerFrameName(size_t sens_index, std::string &frameName) const override;

    /**
     * Get three axis magnetometer measurements
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @param[out] out 3D magnetometer measurement
     * @param[out] timestamp timestamp of measurement
     * @return true/false success/failure
     */
    virtual bool getThreeAxisMagnetometerMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    /* IOrientationSensors methods */
    /**
     * Get the  number of orientation sensors in the device
     * @return 1
     */
    virtual size_t getNrOfOrientationSensors() const override;

    /**
     * Get the status of orientation sensor
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @return MAS_OK/MAS_ERROR if status ok/failure
     */
    virtual yarp::dev::MAS_status getOrientationSensorStatus(size_t sens_index) const override;

    /**
     * Get the name of orientation sensor
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @param[out] name name of the sensor
     * @return true/false success/failure
     */
    virtual bool getOrientationSensorName(size_t sens_index, std::string &name) const override;

    /**
     * Get the name of the frame in which orientation sensor measurements are expressed
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @param[out] frameName name of the sensor frame
     * @return true/false success/failure
     */
    virtual bool getOrientationSensorFrameName(size_t sens_index, std::string &frameName) const override;

    /**
     * Get orientation sensor measurements
     * @param[in] sens_index sensor index (must be 0 in the case xsensmt)
     * @param[out] out RPY Euler angles in deg
     * @param[out] timestamp timestamp of measurement
     * @return true/false success/failure
     */
    virtual bool getOrientationSensorMeasureAsRollPitchYaw(size_t sens_index, yarp::sig::Vector& rpy, double& timestamp) const override;

    virtual yarp::os::Stamp getLastInputStamp() override;

private:
    yarp::dev::MAS_status genericGetStatus(size_t sens_index) const;
    bool genericGetSensorName(size_t sens_index, std::string &name) const;
    bool genericGetFrameName(size_t sens_index, std::string &frameName) const;

    std::string m_sensorName;
    std::string m_frameName;

    struct SensorFrequencies
    {
        double acc{-1};
        double gyro{-1};
        double mag{-1};
        double euler{-1};
        double position{-1};
        double linearVelocity{-1};
    };

    SensorFrequencies m_frequencies;

    const int m_nchannels{12};
    double m_timeoutInSecond{0.1};

    // Interface exposed by the Xsens MT Software suite
    XsControl* m_xsensControl{nullptr};
    XsDevice* m_xsensDevice{nullptr};
    XsPortInfo m_portInfo;
    CallbackHandler m_callback;
};


#endif
