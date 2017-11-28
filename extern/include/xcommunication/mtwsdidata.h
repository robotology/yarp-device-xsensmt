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

#ifndef MTWSDIDATA_H
#define MTWSDIDATA_H

#include <xsens/pstdint.h>
#include <xsens/xsvector3.h>
#include <xsens/xsquaternion.h>
#include <xsens/xsdeviceid.h>

struct XsRange;

struct MtwSdiData
{
	XsDeviceId		m_deviceId;			//!< The ID of the device that generated the data
	uint8_t			m_timeSync;			//!< Indicates if the time sync is in order (unused)
	uint16_t		m_firstFrameNumber;	//!< The first frame number of the SDI interval. The time of the interval is [first, last)
	uint16_t		m_lastFrameNumber;	//!< The last frame number of the SDI interval. The time of the interval is [first, last)
	XsVector3		m_currentBias;		//!< The gyroscope bias used during the SDI interval
	XsQuaternion	m_orientationIncrement;	//!< The orientation increment (delta Q) over the interval
	XsVector3		m_velocityIncrement;	//!< The velocity increment (delta V) over the interval
	bool			m_aidingData;		//!< reserved
	double			m_barometer;		//!< The barometer value during the interval
	XsVector3		m_magnetoMeter;		//!< The magnetometer values during the interval
	int8_t			m_rssi;				//!< The Received Signal Strength Indication (RSSI) of the message

	MtwSdiData();
	MtwSdiData(const MtwSdiData& other);
	~MtwSdiData();
	const MtwSdiData& operator=(const MtwSdiData& other);

	bool empty() const;
	bool containsAidingData() const;
	XsQuaternion orientationIncrement() const;
	XsVector velocityIncrement() const;
	double pressure() const;
	XsVector magneticField() const;
	XsVector currentBias() const;
	XsRange frameRange() const;
	double rssi() const;
};

#endif
