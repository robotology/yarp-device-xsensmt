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

#include <xsens/xstypesconfig.h>
#include "int_xsdatapacket.h"
#include "legacydatapacket.h"
#include "packetfixeddata.h"
#include "mtwsdidata.h"
#include <xsens/xsbusid.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xsmessage.h>
#include <xsens/xsgpspvtdata.h>

/*! \class XsDataPacket
	\brief Contains data received from a device or read from a file

	This class is used by XDA for easy access to data contained in a message. It analyzes its internal
	XsMessage upon contruction to give access to the individual contained pieces of data.
	New data can also be added to the %XsDataPacket or updated if it already existed.
*/
extern "C" {

void validatePacket(XsDataPacket* thisPtr);

void XsDataPacket_assignFromLegacyDataPacket(struct XsDataPacket* thisPtr, struct LegacyDataPacket const* legacy, int index)
{
	assert(legacy);

	if (!legacy->isXbusSystem())
		index = 0;

	thisPtr->clear();

	thisPtr->setDeviceId(legacy->deviceId(index));
	thisPtr->setTimeOfArrival(legacy->timeOfArrival());
	thisPtr->setPacketId(legacy->largePacketCounter());
	XsDataFormat format = legacy->dataFormat(index);
	if (legacy->containsRawData(index))
		thisPtr->setRawData(legacy->rawData(index));
	if (legacy->rawTemperatureChannelCount(index) == 4)
	{
		XsUShortVector rawGyroTemperatures;
		rawGyroTemperatures[0] = legacy->rawTemperature(index, 1);
		rawGyroTemperatures[1] = legacy->rawTemperature(index, 2);
		rawGyroTemperatures[2] = legacy->rawTemperature(index, 3);
		thisPtr->setRawGyroscopeTemperatureData(rawGyroTemperatures);
	}
	if (legacy->containsCalibratedAcceleration(index))
		thisPtr->setCalibratedAcceleration(legacy->calibratedAcceleration(index));
	if (legacy->containsCalibratedGyroscopeData(index))
	{
		//\todo Fix legacy watermarking
//		XsDataIdentifier di;
//		switch (format.m_outputSettings & XOS_Dataformat_Mask) {
//		case XOS_Dataformat_Float:
//			di = XDI_SubFormatFloat;
//			break;
//		case XOS_Dataformat_Double:
//			di = XDI_SubFormatDouble;
//			break;
//		case XOS_Dataformat_Fp1632:
//			di = XDI_SubFormatFp1632;
//			break;
//		case XOS_Dataformat_F1220:
//			di = XDI_SubFormatFp1220;
//			break;
//		default:
//			di = XDI_None;
//			break;
//		}
		// Special copy to preserve watermarking
		//thisPtr->message().setDataShort(XDI_RateOfTurn | di, thisPtr->message().getDataSize());
		//thisPtr->message().setDataByte(3*thisPtr->getFPValueSize(di), thisPtr->message().getDataSize());
		//thisPtr->itemCount()++;
		//thisPtr->message().setDataBuffer(thisPtr->legacyMsg().getDataBuffer(info.m_calGyr), 3*thisPtr->getFPValueSize(di), XsDataPacket_itemOffsetExact(thisPtr, XDI_RateOfTurn | di));
		thisPtr->setCalibratedGyroscopeData(legacy->calibratedGyroscopeData(index));
	}
	if (legacy->containsCalibratedMagneticField(index))
		thisPtr->setCalibratedMagneticField(legacy->calibratedMagneticField(index));
	if (legacy->containsOrientationQuaternion(index))
		thisPtr->setOrientationQuaternion(legacy->orientationQuaternion(index), (format.m_outputSettings & XOS_Coordinates_Ned) ? XDI_CoordSysNed : XDI_CoordSysNwu);
	if (legacy->containsOrientationEuler(index))
		thisPtr->setOrientationEuler(legacy->orientationEuler(index), (format.m_outputSettings & XOS_Coordinates_Ned) ? XDI_CoordSysNed : XDI_CoordSysNwu);
	if (legacy->containsOrientationMatrix(index))
		thisPtr->setOrientationMatrix(legacy->orientationMatrix(index), (format.m_outputSettings & XOS_Coordinates_Ned) ? XDI_CoordSysNed : XDI_CoordSysNwu);
	if (legacy->containsPositionLLA(index))
		thisPtr->setPositionLLA(legacy->positionLLA(index));
	if (legacy->containsVelocity(index))
		thisPtr->setVelocity(legacy->velocity(index), (format.m_outputSettings & XOS_Coordinates_Ned) ? XDI_CoordSysNed : XDI_CoordSysNwu);
	if (legacy->containsStatus(index))
	{
		bool isDetailed = true;
		uint32_t status = legacy->status(index, &isDetailed);

		// For MTw's only, the status needs to be swapEndian32-ed
		if (thisPtr->deviceId().isMtw1())
		{
			status = swapEndian32(status);
		}

		if (isDetailed)
		{
			thisPtr->setStatus(status);
		}
		else
		{
			thisPtr->setStatusByte((uint8_t) status);
		}
	}
	if (legacy->containsGpsPvtData())
	{
		XsPressure tmp;
		tmp.m_pressure = legacy->gpsPvtData(index).m_pressure * 2.0; // Convert to Pascal
		tmp.m_pressureAge = legacy->gpsPvtData(index).m_pressureAge;
		thisPtr->setPressure(tmp);
		XsGpsPvtData pvt;
		pvt = legacy->gpsPvtData(index);
		thisPtr->setGpsPvtData(pvt);
	}
	if (legacy->containsUtcTime(index))
	{
		XsUtcTime time = legacy->utcTime(index);
		thisPtr->setUtcTime(time);
	}
	if (legacy->containsSampleTimeFine(index))
	{
		uint32_t sampleTimeFine = legacy->sampleTimeFine(index);
		thisPtr->setSampleTimeFine(sampleTimeFine);
	}

	// packet counter MUST go before anything that sets a frame range!
	if (legacy->containsPacketCounter(index))
		thisPtr->setPacketCounter(legacy->packetCounter(index));

	if (legacy->containsMtwSdiData(index))
	{
		// not yet converted:
		//uint8_t			m_timeSync;
		//XsVector3		m_currentBias;

		MtwSdiData mtwsdi = legacy->mtwSdiData(index);
		thisPtr->setDeviceId(mtwsdi.m_deviceId);
		{
			XsSdiData tmp(mtwsdi.orientationIncrement(), mtwsdi.velocityIncrement());
			thisPtr->setSdiData(tmp);

			//// Special copy to preserve watermarking
			//thisPtr->message().setDataShort(XDI_DeltaQ | di, thisPtr->message().getDataSize());
			//thisPtr->message().setDataByte(4*thisPtr->getFPValueSize(di), thisPtr->message().getDataSize());
			//thisPtr->itemCount()++;
			//thisPtr->message().setDataBuffer(thisPtr->legacyMsg().getDataBuffer(info.m_wOrientationIncrement), 4*thisPtr->getFPValueSize(di), XsDataPacket_itemOffsetExact(thisPtr, XDI_DeltaQ | di));

			//thisPtr->message().setDataShort(XDI_DeltaV | di, thisPtr->message().getDataSize());
			//thisPtr->message().setDataByte(3*thisPtr->getFPValueSize(di), thisPtr->message().getDataSize());
			//thisPtr->itemCount()++;
			//thisPtr->message().setDataBuffer(thisPtr->legacyMsg().getDataBuffer(info.m_wVelocityIncrement), 3*thisPtr->getFPValueSize(di), XsDataPacket_itemOffsetExact(thisPtr, XDI_DeltaV | di));
		}

		//if (mtwsdi.m_aidingData)
		thisPtr->setCalibratedMagneticField(mtwsdi.m_magnetoMeter);
		if (mtwsdi.m_barometer)
		{
			XsPressure tmp;
			tmp.m_pressure = mtwsdi.m_barometer * 100.0; // convert from millibar to pascal
			tmp.m_pressureAge = mtwsdi.m_barometer?0:255;
			thisPtr->setPressure(tmp);
		}
		thisPtr->setFrameRange(XsRange((int) mtwsdi.m_firstFrameNumber, (int) mtwsdi.m_lastFrameNumber));
		thisPtr->setRssi(mtwsdi.m_rssi);
	}
	if (legacy->containsTemperature(index))
		thisPtr->setTemperature(legacy->temperature(index));

	// enable this when you experience weird crashes in XsByteArray_destruct or XsDataPacket_destruct
	//validatePacket(thisPtr);
}

} // extern "C"
