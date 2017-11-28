/*	WARNING: COPYRIGHT (C) 2017 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
	THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
	FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
	TO A RESTRICTED LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
	LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
	INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
	DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
	IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
	USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
	XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
	OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
	COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
*/

#ifndef XSDATAIDENTIFIER_H
#define XSDATAIDENTIFIER_H

#define XDI_MAX_FREQUENCY		((uint16_t) 0xFFFF)

//////////////////////////////////////////////////////////////////////////////////////////
/*!	\addtogroup enums Global enumerations
	@{
*/

/*!	\enum XsDataIdentifier
	\brief Defines the data identifiers

	The list of standard data identifiers is shown below.
	The last positions in the data identifier depends on the configuration of the data.
	For example 0x4020 is 3D acceleration in single precision float format,
	where 0x4022 is 3D acceleration in fixed point 16.32 format.

	Refer to the Low Level Communication Protocol for more details.
*/
enum XsDataIdentifier
{
	XDI_None					= 0x0000,	//!< Empty datatype
	XDI_TypeMask				= 0xFE00,	//!< Mask for checking the group which a dataidentifier belongs to, Eg. XDI_TimestampGroup or XDI_OrientationGroup
	XDI_FullTypeMask			= 0xFFF0,	//!< Mask to get the type of data, without the data format
	XDI_FullMask				= 0xFFFF,	//!< Complete mask to get entire data identifier
	XDI_FormatMask				= 0x01FF,	//!< Mask for getting the data id without checking the group
	XDI_DataFormatMask			= 0x000F,	//!< Mask for extracting just the data format /sa XDI_SubFormat

	XDI_SubFormatMask			= 0x0003,	//!< Determines, float, fp12.20, fp16.32, double output... (where applicable)
	XDI_SubFormatFloat			= 0x0000,	//!< Floating point format
	XDI_SubFormatFp1220			= 0x0001,	//!< Fixed point 12.20
	XDI_SubFormatFp1632			= 0x0002,	//!< Fixed point 16.32
	XDI_SubFormatDouble			= 0x0003,	//!< Double format

	XDI_TemperatureGroup		= 0x0800,	//!< Group for temperature outputs
	XDI_Temperature				= 0x0810,	//!< Temperature

	XDI_TimestampGroup			= 0x1000,	//!< Group for time stamp related outputs
	XDI_UtcTime					= 0x1010,	//!< Utc time from the GPS receiver
	XDI_PacketCounter			= 0x1020,	//!< Packet counter, increments every packet
	XDI_Itow					= 0x1030,	//!< Itow. Time Of Week from the GPS receiver
	XDI_GpsAge					= 0x1040,	//!< Age of Gps sample \deprecated Replaced by XDI_GnssAge
	XDI_GnssAge					= 0x1040,	//!< Gnss age from the GPS receiver
	XDI_PressureAge				= 0x1050,	//!< Age of a pressure sample, in packet counts
	XDI_SampleTimeFine			= 0x1060,	//!< Sample Time Fine
	XDI_SampleTimeCoarse		= 0x1070,	//!< Sample Time Coarse
	XDI_FrameRange				= 0x1080,	//!< Reserved \internal add for MTw (if needed)
	XDI_PacketCounter8			= 0x1090,	//!< 8 bit packet counter, wraps at 256
	XDI_SampleTime64			= 0x10A0,	//!< 64 bit sample time

	XDI_OrientationGroup		= 0x2000,	//!< Group for orientation related outputs
	XDI_CoordSysMask			= 0x000C,	//!< Mask for the coordinate system part of the orientation data identifier
	XDI_CoordSysEnu				= 0x0000,	//!< East North Up orientation output
	XDI_CoordSysNed				= 0x0004,	//!< North East Down orientation output
	XDI_CoordSysNwu				= 0x0008,	//!< North West Up orientation output
	XDI_Quaternion				= 0x2010,	//!< Orientation in quaternion format
	XDI_RotationMatrix			= 0x2020,	//!< Orientation in rotation matrix format
	XDI_EulerAngles				= 0x2030,	//!< Orientation in euler angles format

	XDI_PressureGroup			= 0x3000,	//!< Group for pressure related outputs
	XDI_BaroPressure			= 0x3010,	//!< Pressure output recorded from the barometer

	XDI_AccelerationGroup		= 0x4000,	//!< Group for acceleration related outputs
	XDI_DeltaV					= 0x4010,	//!< DeltaV SDI data output
	XDI_Acceleration			= 0x4020,	//!< Acceleration output in m/s2
	XDI_FreeAcceleration		= 0x4030,	//!< Free acceleration output in m/s2
	XDI_AccelerationHR			= 0x4040,	//!< AccelerationHR output

	XDI_PositionGroup			= 0x5000,	//!< Group for position related outputs
	XDI_AltitudeMsl				= 0x5010,	//!< Altitude at Mean Sea Level
	XDI_AltitudeEllipsoid		= 0x5020,	//!< Altitude at ellipsoid
	XDI_PositionEcef			= 0x5030,	//!< Position in earth-centered, earth-fixed format
	XDI_LatLon					= 0x5040,	//!< Position in latitude, longitude

	XDI_SnapshotGroup			= 0xC800,	//!< Group for snapshot related outputs
	XDI_RetransmissionMask		= 0x0001,	//!< Mask for the retransmission bit in the snapshot data
	XDI_RetransmissionFlag		= 0x0001,	//!< Bit indicating if the snapshot if from a retransmission
	XDI_AwindaSnapshot 			= 0xC810,	//!< Awinda type snapshot
	XDI_FullSnapshot 			= 0xC820,	//!< Full snapshot

	XDI_GnssGroup				= 0x7000,	//!< Group for Gnss related outputs
	XDI_GnssPvtData				= 0x7010,	//!< Gnss position, velocity and time data
	XDI_GnssSatInfo				= 0x7020,	//!< Gnss satellite information

	XDI_AngularVelocityGroup	= 0x8000,	//!< Group for angular velocity related outputs
	XDI_RateOfTurn				= 0x8020,	//!< Rate of turn data in rad/sec
	XDI_DeltaQ					= 0x8030,	//!< DeltaQ SDI data
	XDI_RateOfTurnHR			= 0x8040,	//!< Rate of turn HR data

	XDI_GpsGroup				= 0x8800,	//!< Group for GPS only related data \deprecated Replaced by XDI_GnssGroup
	XDI_GpsDop					= 0x8830,	//!< Gps dilution of precision data \deprecated
	XDI_GpsSol					= 0x8840,	//!< Gps navigation solution information \deprecated Replaced by XDI_GnssPvtData
	XDI_GpsTimeUtc				= 0x8880,	//!< Gps time in UTC format \deprecated Replaced by XDI_GnssPvtData
	XDI_GpsSvInfo				= 0x88A0,	//!< Gps satellite vehicle information \deprecated Replaced by XDI_GnssSatInfo

	XDI_RawSensorGroup			= 0xA000,	//!< Group for raw sensor data related outputs
	XDI_RawUnsigned				= 0x0000,	//!< Tracker produces unsigned raw values, usually fixed behavior
	XDI_RawSigned				= 0x0001,	//!< Tracker produces signed raw values, usually fixed behavior
	XDI_RawAccGyrMagTemp		= 0xA010,	//!< Raw acceleration, gyroscope, magnetometer and temperature data
	XDI_RawGyroTemp				= 0xA020,	//!< Raw gyroscope and temperature data
	XDI_RawAcc					= 0xA030,	//!< Raw acceleration data
	XDI_RawGyr					= 0xA040,	//!< Raw gyroscope data
	XDI_RawMag					= 0xA050,	//!< Raw magnetometer data
	XDI_RawDeltaQ				= 0xA060,	//!< Raw deltaQ SDI data
	XDI_RawDeltaV				= 0xA070,	//!< Raw deltaV SDI data
	XDI_RawBlob					= 0xA080,	//!< Raw blob data

	XDI_AnalogInGroup			= 0xB000,	//!< Group for analog in related outputs
	XDI_AnalogIn1				= 0xB010,	//!< Data containing adc data from analog in 1 line (if present)
	XDI_AnalogIn2				= 0xB020,	//!< Data containing adc data from analog in 2 line (if present)

	XDI_MagneticGroup			= 0xC000,	//!< Group for magnetometer related outputs
	XDI_MagneticField			= 0xC020,	//!< Magnetic field data in a.u.

	XDI_VelocityGroup			= 0xD000,	//!< Group for velocity related outputs
	XDI_VelocityXYZ				= 0xD010,	//!< Velocity in XYZ coordinate frame

	XDI_StatusGroup				= 0xE000,	//!< Group for status related outputs
	XDI_StatusByte				= 0xE010,	//!< Status byte
	XDI_StatusWord				= 0xE020,	//!< Status word
	XDI_Rssi					= 0xE040,	//!< Rssi information
	XDI_DeviceId				= 0xE080,	//!< DeviceId output

	XDI_IndicationGroup			= 0x4800,	//!< 0100.1000 -> bit reverse = 0001.0010 -> type 18
	XDI_TriggerIn1				= 0x4810,	//!< Trigger in 1 indication
	XDI_TriggerIn2				= 0x4820,	//!< Trigger in 2 indication
};
/*! @} */

typedef enum XsDataIdentifier XsDataIdentifier;

#ifdef __cplusplus
inline XsDataIdentifier operator | (XsDataIdentifier a, XsDataIdentifier b)
{
	return (XsDataIdentifier) ((int) a | (int) b);
}

inline XsDataIdentifier operator & (XsDataIdentifier a, XsDataIdentifier b)
{
	return (XsDataIdentifier) ((int) a & (int) b);
}

inline XsDataIdentifier operator ~ (XsDataIdentifier a)
{
	return (XsDataIdentifier) ~((unsigned short)a);
}
#endif

#include "xsdataidentifiervalue.h"

#endif
