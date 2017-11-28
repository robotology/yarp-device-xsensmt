/*	Copyright (c) 2003-2017 Xsens Technologies B.V. or subsidiaries worldwide.
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1.	Redistributions of source code must retain the above copyright notice,
		this list of conditions and the following disclaimer.

	2.	Redistributions in binary form must reproduce the above copyright notice,
		this list of conditions and the following disclaimer in the documentation
		and/or other materials provided with the distribution.

	3.	Neither the names of the copyright holders nor the names of their contributors
		may be used to endorse or promote products derived from this software without
		specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
	EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
	THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
	SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
	OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
	HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
	TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <xsens/xsrange.h>
#include <xsens/xsrssi.h>
#include "mtwsdidata.h"

/*!
	\class MtwSdiData
	\brief Class to store strapdown integration data.
	\note Intended for internal use.
*/

/*! \brief Construct an empty strapdown integration data object */
MtwSdiData::MtwSdiData()
: m_deviceId(0)
, m_timeSync(0)
, m_firstFrameNumber(0)
, m_lastFrameNumber(0)
, m_aidingData(0)
, m_barometer(0)
, m_rssi(0)
{
}

/*! \brief Construct a copy of \a other */
MtwSdiData::MtwSdiData(const MtwSdiData &other)
: m_deviceId(other.m_deviceId)
, m_timeSync(other.m_timeSync)
, m_firstFrameNumber(other.m_firstFrameNumber)
, m_lastFrameNumber(other.m_lastFrameNumber)
, m_currentBias(other.m_currentBias)
, m_orientationIncrement(other.m_orientationIncrement)
, m_velocityIncrement(other.m_velocityIncrement)
, m_aidingData(other.m_aidingData)
, m_barometer(other.m_barometer)
, m_magnetoMeter(other.m_magnetoMeter)
, m_rssi(other.m_rssi)
{
}

/*! \brief Destroy the strapdown integration data structure. */
MtwSdiData::~MtwSdiData()
{
}

/*! \brief Assign \a other to this. */
const MtwSdiData& MtwSdiData::operator=(const MtwSdiData& other)
{
	if (this == &other)
		return *this;
	m_deviceId = other.m_deviceId;
	m_timeSync = other.m_timeSync;
	m_firstFrameNumber = other.m_firstFrameNumber;
	m_lastFrameNumber = other.m_lastFrameNumber;
	m_currentBias = other.m_currentBias;
	m_orientationIncrement = other.m_orientationIncrement;
	m_velocityIncrement = other.m_velocityIncrement;
	m_aidingData = other.m_aidingData;
	m_barometer = other.m_barometer;
	m_magnetoMeter = other.m_magnetoMeter;
	m_rssi = other.m_rssi;
	return *this;
}


/*! \brief Test if this is a null Awinda object. */
bool MtwSdiData::empty() const
{
	return !m_deviceId.toInt();
}

/*! \brief Test if strapdown integration data is available. */
bool MtwSdiData::containsAidingData() const
{
	if (empty())
		return false;
	return m_aidingData;
}

/*! \brief Get the orientation increment value. */
XsQuaternion MtwSdiData::orientationIncrement() const
{
	if (empty())
		return XsQuaternion();
	return m_orientationIncrement;
}

/*! \brief Get the velocity increment value. */
XsVector MtwSdiData::velocityIncrement() const
{
	if (empty())
		return XsVector();
	return m_velocityIncrement;
}

/*! \brief Get the pressure as measured by the barometer in hPa. */
double MtwSdiData::pressure() const
{
	if (empty())
		return 0;
	return m_barometer;
}

/*! \brief Get the magnetic field value. */
XsVector MtwSdiData::magneticField() const
{
	if (empty())
		return XsVector();
	return m_magnetoMeter;
}

/*! \brief Get the current gyroscope bias value. */
XsVector MtwSdiData::currentBias() const
{
	if (empty())
		return XsVector();
	return m_currentBias;
}

/*! \brief Get the frame range of the current strapdown integration data. */
XsRange MtwSdiData::frameRange() const
{
	if (empty())
		return XsRange();
	return XsRange(m_firstFrameNumber, m_lastFrameNumber);
}

/*! \brief Get the rssi of the received strapdown integration data */
double MtwSdiData::rssi() const
{
	if(empty())
		return XS_RSSI_UNKNOWN;
	return (double)m_rssi;
}

